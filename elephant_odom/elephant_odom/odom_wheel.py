#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

from std_msgs.msg import Float32MultiArray
import casadi as ca
from casadi import sin, cos, pi, arctan2

step_horizon = 0.02         # 50 Hz time between steps in seconds
wheel_radius = 0.0622       # wheel radius
L = 0.37                    # L in J Matrix (half robot x-axis length)

# specs
x_init = 0.0
y_init = 0.0
theta_init = 0.0

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
class Odom_Wheel(Node):
    def __init__(self):
        # Init node 
        super(Odom_Wheel, self).__init__('odom_wheel')
        self.create_timer(step_horizon,self.operation_run)  ## 50 Hz
        self.subscsriber_wheel = self.create_subscription(Float32MultiArray, 'feedback',self.msg_sub, 10)
        self.publish_wheel_odom = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.init_param()
        ## 
    
    def msg_sub(self,msg):
        self.u = ca.DM([msg.data[0], msg.data[1], msg.data[2], msg.data[3]])

    def operation_run(self):
        self.state_init, self.speed_init = self.shift_timestep(step_horizon,self.state_init,self.u,self.f)
        yaw = arctan2(sin(self.state_init[2]), cos(self.state_init[2]))
        q = quaternion_from_euler(0.0,0.0,yaw)
        ## 
        quatOdom = Odometry()
        quatOdom.header.stamp = self.get_clock().now().to_msg()
        quatOdom.header.frame_id = 'wheel_odom'
        quatOdom.pose.pose.position.x = (float)(self.state_init[0])
        quatOdom.pose.pose.position.y = (float)(self.state_init[1])
        ## 
        quatOdom.pose.pose.orientation.x = q[0]
        quatOdom.pose.pose.orientation.y = q[1]
        quatOdom.pose.pose.orientation.z = q[2]
        quatOdom.pose.pose.orientation.w = q[3]
        ## 
        quatOdom.twist.twist.linear.x = (float)(self.speed_init[0])
        quatOdom.twist.twist.linear.y = (float)(self.speed_init[1])
        ## 
        quatOdom.twist.twist.angular.z = (float)(self.speed_init[2])

        for i in range (36):
            if(i == 0 or i == 7 or i == 14):
                quatOdom.pose.covariance[i] = 0.01
            elif (i == 21 or i == 28 or i== 35):
                quatOdom.pose.covariance[i] += 0.1
            else :
                quatOdom.pose.covariance[i] = 0
        
        self.get_logger().info("pub!!")
        print(self.state_init)
        self.publish_wheel_odom.publish(quatOdom)
        

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
        n_states = states.numel()

        ## control symbolic variables
        V_a = ca.SX.sym('V_a')
        V_b = ca.SX.sym('V_b')
        V_c = ca.SX.sym('V_c')
        V_d = ca.SX.sym('V_d')
        controls = ca.vertcat(
            V_a,
            V_b,
            V_c,
            V_d
        )
        n_controls = controls.numel()

        ## rotation matrix
        rot_3d_z = ca.vertcat(
            ca.horzcat(cos(theta), -sin(theta), 0),
            ca.horzcat(sin(theta),  cos(theta), 0),
            ca.horzcat(         0,           0, 1)
        )

        J = (wheel_radius/(4*0.707)) * ca.DM([
            [         1,         1,          -1,         -1],
            [        1,         -1,          -1,        1],
            [1/(2*L), 1/(2*L), 1/(2*L), 1/(2*L)]
        ])

        self.state_init = ca.DM([x_init, y_init, theta_init])        # initial state
        self.speed_init = ca.DM([0.0, 0.0, 0.0])        # initial state

        RHS = rot_3d_z @ J @ controls

        ## maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T
        self.f = ca.Function('f', [states, controls], [RHS])

        self.u = ca.DM([0.0, 0.0, 0.0, 0.0])

        ## integral for position 
    def shift_timestep(self, step_horizon, state_init, u, f):
        f_value = f(state_init, u)
        next_state = ca.DM.full(state_init + (step_horizon * f_value))
        return next_state, f_value

def main(args=None):
    rclpy.init(args=args)
    odom_wheel = Odom_Wheel()
    rclpy.spin(odom_wheel)
    odom_wheel.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    try :
        main()
    except KeyboardInterrupt :
        pass