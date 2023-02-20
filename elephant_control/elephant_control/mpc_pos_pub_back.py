import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from test_mpc import cubic_spline_planner
import math
import casadi as ca
##########################

# trajactory
def get_straight_course(dl):
    ax = [10.38,10.38,10.3,10.3,10.0,6.0,3.0,0.0]
    ay = [4.14,2.5,1.58,0.21,0.0,0.0,0.0,0.0]
    
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
def get_switch_back_course(dl):

    ax = [10.08, 10.21, 10 ]
    ay = [4.14, 1.58, 0.21]
    ax2 = [10 ,0.0 ]
    ay2 = [0.21, 0.0 ]
    # ax = [0.0, 1.0, 1.99, 4.0, 5.55,6.76] # 
    # ay = [0.0, -1.66,-2.92, -2.53, -1.8, 0.24] # 
    # ax = [6.76, 5.55,4.0, 1.99, 1.0, 0.0] #  
    # ay = [ 0.24, -1.8,-2.53, -2.92, -1.66, 0.0] # 
    # ax = [4.0, 1.99, 1.0, 0.0] # 6.76, 5.55, 
    # ay = [-2.53, -2.92, -1.66, 0.0] # 0.24, -1.8, 
    # ax = [0.0, 1.8, 0.0,-1.8,0.0]
    # ay = [0.0, 1.8, 3.6,1.8,0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax2, ay2, ds=dl)
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)
    return cx, cy, cyaw, ck

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

class pub_class(Node):
    def __init__(self):
        super().__init__('pub_node_test')
        timer_period = 0.08 # seconds
        self.prosses = self.create_timer(timer_period, self.prosses_callback)
        self.pub_timer = self.create_timer(0.05, self.pub_timer_callback)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mpc_position', 100)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.feedback_callback,
            10)
        self.cx, self.cy, self.cyaw, ck = get_straight_course(0.1)

        ############
        self.tick = 0
        self.pub_x = self.cx[0]
        self.pub_y = self.cy[0]
        self.pub_yaw = 0
        self.slow_speed = 1.0
        ############
        self.current_x = 0
        self.current_y = 0
        self.current_yaw = 0
        
    def feedback_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        ox =  msg.pose.pose.orientation.x
        oy =  msg.pose.pose.orientation.y
        oz =  msg.pose.pose.orientation.z
        ow =  msg.pose.pose.orientation.w
        row, pitch , yaw = euler_from_quaternion(ox,oy,oz,ow)
        self.current_yaw = yaw
    def pub_timer_callback(self):
        msg = Float32MultiArray()
        if self.tick <= len(self.cx) - 1 : 
            self.pub_x = self.cx[self.tick]
            self.pub_y = self.cy[self.tick]
            # self.pub_yaw = self.cyaw[self.tick]
        if self.tick > 5 : self.slow_speed = 0.0
        msg.data = [(float) (self.pub_x), (float) (self.pub_y), (float) (self.pub_yaw),(float) (self.slow_speed)]
        self.publisher_.publish(msg)

    def prosses_callback(self):
        self.state_init = ca.DM([self.current_x, self.current_y, self.current_yaw])
        self.state_target = ca.DM([self.pub_x, self.pub_y, self.pub_yaw])
        if( ca.norm_2(self.state_init - self.state_target) <= 2) : 
            self.tick = self.tick + 1


def main(args=None):
    rclpy.init(args=args)

    pub_node_test = pub_class()

    rclpy.spin(pub_node_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_node_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()