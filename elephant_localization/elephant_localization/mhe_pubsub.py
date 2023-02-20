import rclpy
from rclpy.node import Node
from model.mhe import *
import casadi as ca

# ROS topic
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('mhe_node')
        self.elephant = ElephantModel()
        self.timer = 0.01
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.q = np.zeros(4)
        self.wheel_vel = np.zeros(4)
        self.wheel_cal = np.zeros(4)
        self.velocity_callback = np.zeros(4)
        self.command_vel = np.zeros(4)
        self.position = np.zeros(3)
        self.control_n = np.zeros(4)
        self.dt = 0.01

        # initialization
        self.Nmhe = 10
        self.nx = 3
        self.nu = 4
        self.sampling_time = 0.1
        self.t0 = 0
        self.sim_time = 20
        self.mheiter = 0
        N = 10
        self.current_state = np.array([0.0, 0.0, 0.0])
        self.current_control = np.array([0.0, 0.0, 0.0, 0.0])
        self.control = np.tile(self.current_control.reshape(-1,1), self.Nmhe).T
        self.state = np.tile(self.current_state.reshape(-1,1), self.Nmhe+1).T


        # == Estimation matrix for measurement
        self.X_mhe = np.zeros((self.Nmhe+1, self.nx))
        self.U_mhe = np.zeros((self.Nmhe, self.nu))
        self.U_est = np.zeros((self.Nmhe,self.nu))

		# === Export MHE ===
        self.f_mhe, self.solver_mhe, self.args_mhe = casadi_mhe_model(self.nx, self.nu, self.Nmhe).casadi_setup_mhe()
		# === Measurement Model ===

        self.x_est = np.zeros((self.Nmhe+1,1))
        self.y_est = np.zeros((self.Nmhe+1,1))
        self.theta_est = np.zeros((self.Nmhe+1,1))

		# === Add Noise ===
        self.x_noise = np.random.normal(0.001,0.001)
        self.y_noise = np.random.normal(0.001,0.001)
        self.theta_noise = np.random.normal(np.radians(2), np.radians(0.02))
        self.noise = np.array([self.x_noise, self.y_noise, self.theta_noise]).reshape(1,-1)

        # ROS2 Publisher and Subscriber
        self.imu_subscriber = self.create_subscription(Imu, "/imu/data2", self.imu_callback, 100)
        self.control_feedback = self.create_subscription(Float32MultiArray, "feedback", self.control_callback, 100)
        self.timer_integral = self.create_timer(self.timer, self.integral)
        self.odom_publisher = self.create_publisher(Vector3, "/odom/data",10)
        self.odom_timer = self.create_timer(self.timer, self.odom_callback)


    def control_callback(self, msg):
        self.wheel_vel[0] = msg.data[0]
        self.wheel_vel[1] = msg.data[1]
        self.wheel_vel[2] = msg.data[2]
        self.wheel_vel[3] = msg.data[3]
        self.control_n = self.wheel_vel

        self.command_vel[0], self.command_vel[1], self.command_vel[2] = self.elephant.forward_kinematic(self.wheel_vel[0], self.wheel_vel[1], self.wheel_vel[2], self.wheel_vel[3], 0.0)

    def imu_callback(self, imu_msg):
        self.q[0] = imu_msg.orientation.x
        self.q[1] = imu_msg.orientation.y
        self.q[2] = imu_msg.orientation.z
        self.q[3] = imu_msg.orientation.w

        self.roll, self.pitch, self.yaw = self.elephant.euler_from_quaternion(self.q[0], self.q[1], self.q[2], self.q[3])
        self.yaw = -self.yaw

    def integral(self):
        dx = (self.command_vel[0]*np.cos(self.yaw) + self.command_vel[1]*np.sin(self.yaw))*self.dt 
        dy = (-self.command_vel[0]*np.sin(self.yaw) + self.command_vel[1]*np.cos(self.yaw))*self.dt
        self.position[0] = dx + self.position[0]
        self.position[1] = dy + self.position[1]
        self.current_state = np.array([self.position[0], self.position[1],self.yaw])

    def odom_callback(self):
        odom_msg = Vector3()
        self.current_control = self.control_n
		# while (self.mheiter * self.sampling_time < self.sim_time):
        for k in range(self.Nmhe+1):
            self.x_est[k] = self.current_state[0]
            self.y_est[k] = self.current_state[1]
            self.theta_est[k] = self.current_state[2]

        self.y_measurement = np.concatenate([self.x_est, self.y_est, self.theta_est], axis=1)
        for k in range(self.Nmhe+1):
            self.X_mhe[k,0] = self.x_est[k]
            self.X_mhe[k,1] = self.y_est[k]
            self.X_mhe[k,2] = self.theta_est[k]

        for k in range(self.Nmhe):
            self.U_est[k,0] = self.control[:self.Nmhe][k, 0]
            self.U_est[k,1] = self.control[:self.Nmhe][k, 1]
            self.U_est[k,2] = self.control[:self.Nmhe][k, 2]
            self.U_est[k,3] = self.control[:self.Nmhe][k, 3]
        for k in range(self.y_measurement.shape[0]-self.Nmhe):
            self.args_mhe['p'] = np.concatenate([
				self.y_measurement.reshape(-1,1),
				self.U_est.reshape(-1,1)
			])
            self.args_mhe['x0'] = np.concatenate([
				self.X_mhe.reshape(-1,1),
				self.U_mhe.reshape(-1,1)
			])
            self.sol_mhe = self.solver_mhe(
				x0 = self.args_mhe['x0'],
				p = self.args_mhe['p'],
				lbx = self.args_mhe['lbx'],
				ubx = self.args_mhe['ubx'],
				lbg = self.args_mhe['lbg'],
				ubg = self.args_mhe['ubg']
			)
            self.sol_Xmhe = ca.reshape(self.sol_mhe['x'][:self.nx*(self.Nmhe+1)],  self.nx, self.Nmhe+1)
            self.sol_Umhe = ca.reshape(self.sol_mhe['x'][self.nx*(self.Nmhe+1):], self.nu, self.Nmhe)
            self.X_mhe = np.concatenate([self.sol_Xmhe[:, 1:], self.sol_Xmhe[:,-1:]],axis=1).T
            self.U_mhe = np.concatenate([self.sol_Umhe[:, 1:], self.sol_Umhe[:,-1:]],axis=1).T
        print(np.round(self.sol_Xmhe.full()[:,self.Nmhe],3))
        odom_msg.x = float(np.round(self.sol_Xmhe[0],3))
        odom_msg.y = float(np.round(self.sol_Xmhe[1],3))
        odom_msg.z = float(np.round(self.sol_Xmhe[2],3))
        self.odom_publisher.publish(odom_msg)
        self.current_state = self.sol_Xmhe.full()[:,self.Nmhe]    

def main(args=None):
    rclpy.init(args=args)
    mhe_node = ros_node()
    rclpy.spin(mhe_node)
    mhe_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()