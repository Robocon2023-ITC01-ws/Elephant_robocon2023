import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import casadi as ca
import math
from casadi import sin, cos, pi, arctan2

step_horizon = 0.02         # 50 Hz time between steps in seconds
wheel_radius = 0.03       # wheel radius
L = 0.37                    # L in J Matrix (half robot x-axis length)
# specs
x_init = 0.0
y_init = 0.0
theta_init = 0.0

def euler_from_quaternion(x, y, z, w):
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    
    t3 = 2.0 * (w * z +x * y)
    t4 = 1.0 - 2.0*(y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    #if yaw < 0:
    #	yaw = self.map(yaw, -3.1399, -0.001, 3.1399, 6.2799)
    
    return yaw

def map(Input, min_input, max_input, min_output, max_output):
    value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
    return value 

def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr

    return q

class odom_wheel(Node):
    def __init__(self):
        super().__init__('odom_wheel')
        self.tick_subcriber = self.create_subscription(
            UInt16MultiArray,
            'tick_feedback',
            self.listener_callback,
            20)
        self.imu_subscriber = self.create_subscription(Imu, "/imu/data2", self.imu_callback, 20)
        self.publish_wheel_odom = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.odom_publisher = self.create_publisher(Vector3, "/odom/data",10)
        self.pub_timer = self.create_timer(0.02, self.pub_timer_cb)

        self.ppr = 8200/8      # tick per revelution
        
        self.old_tick = ca.DM([0, 0, 0, 0])
        self.new_tick = ca.DM([0, 0, 0, 0])
        self.diff = ca.DM([0, 0, 0, 0])

        self.first_init = True
        self.covariance_init = True
        self.init_param()

        self.yaw = 0.0

    def imu_callback(self,imu_msg):
        self.yaw = euler_from_quaternion(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
    def pub_timer_cb(self):
        odom_msg = Vector3()
        odom_msg.x = (float)(self.new_state[0])
        odom_msg.y = (float)(self.new_state[1])
        odom_msg.z = (float)(self.yaw)
        self.odom_publisher.publish(odom_msg)

        # quatOdom = Odometry()
        # quatOdom.header.stamp = self.get_clock().now().to_msg()
        # quatOdom.header.frame_id = 'wheel_odom'
        # quatOdom.pose.pose.position.x = (float)(self.new_state[0])
        # quatOdom.pose.pose.position.y = (float)(self.new_state[1])

        # yaw = arctan2(sin(self.new_state[2]), cos(self.new_state[2]))
        
        # q = quaternion_from_euler(0.0,0.0,yaw)
        # ## 
        # quatOdom.pose.pose.orientation.x = q[0]
        # quatOdom.pose.pose.orientation.y = q[1]
        # quatOdom.pose.pose.orientation.z = q[2]
        # quatOdom.pose.pose.orientation.w = q[3]

        # if (self.covariance_init):
        #     self.covariance_init = False
        #     for i in range (36):
        #         if(i == 0 or i == 7 or i == 14):
        #             quatOdom.pose.covariance[i] = 0.01
        #         elif (i == 21 or i == 28 or i== 35):
        #             quatOdom.pose.covariance[i] += 0.1
        #         else :
        #             quatOdom.pose.covariance[i] = 0
        # ##
        # print(self.new_state, yaw)
        # self.publish_wheel_odom.publish(quatOdom)

    def init_param(self):
        ## state symbolic variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(
            x,
            y,
            theta
        )


        ## control symbolic variables
        V_a = ca.SX.sym('V_a')
        V_b = ca.SX.sym('V_b')
        V_c = ca.SX.sym('V_c')
        controls = ca.vertcat(
            V_a,
            V_b,
            V_c
        )

        ## rotation matrix
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )

        J = (wheel_radius) * ca.DM([          ## inverse kinematic
            [-295/312, -295/312, 1],
            [-53/78, 25/78, 0],
            [125/39, 125/39, 0]
        ])

        self.new_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.old_state = ca.DM([x_init, y_init, theta_init])        # initial state
        self.speed_init = ca.DM([0.0, 0.0, 0.0])            # initial state

        RHS = rot_3d_z @ J @ controls
        #RHS = J @ controls

        ## maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        self.f = ca.Function('f', [states, controls], [RHS])

        self.u = ca.DM([0.0, 0.0, 0.0])

    def listener_callback(self, tick_msg):
        if (self.first_init):
            self.first_init = False 

            self.new_tick = ca.DM([(tick_msg.data[0]),(tick_msg.data[1]),(tick_msg.data[2])])   ## first init
            self.old_tick = self.new_tick
        else : 
            self.new_tick = ca.DM([tick_msg.data[0],tick_msg.data[1],tick_msg.data[2]])
            self.diff = self.new_tick - self.old_tick
            ## length tick 
            for i in range(3):
                if (self.diff[i] > 32768):
                    self.diff[i] = self.diff[i] - 65535
                elif (self.diff[i] < -32768):
                    self.diff[i] = self.diff[i] + 65535
            #################
            self.old_state[2] = self.yaw
            self.new_state = self.shift_timestep(self.old_state,self.diff,self.f)
            self.old_state = self.new_state
            #################


        self.old_tick = self.new_tick

    def shift_timestep(self , state_init, u, f):
        f_value = f(state_init, (u* 2* pi)/self.ppr) + state_init
        return f_value




def main(args=None):
    rclpy.init(args=args)

    odom_test = odom_wheel()

    rclpy.spin(odom_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
