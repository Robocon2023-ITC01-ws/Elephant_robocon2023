import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3

try :
    from elephant_publish_pos import cubic_spline_planner
except :
    pass
try :
    import cubic_spline_planner
except :
    pass
import math
import casadi as ca
##########################

# trajactory
def get_straight_course(dl):      # use for two point
    ax = [0.0 ,3.0,6.0,10.0,10.3,10.3,10.38,10.38]
    ay = [0.0, 0.0,0.0,0.0,0.21,1.58,2.5,4.14]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
###################################
def get_switch_back_course(dl):
    ax = [0.0 , 10 ]
    ay = [0.0, 0.21]
    ax2 = [10, 10.21, 10.08]
    ay2 = [0.21, 1.58, 4.14]
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
        timer_period = 0.05 # seconds
        self.prosses = self.create_timer(timer_period, self.prosses_callback)
        self.pub_timer = self.create_timer(0.05, self.pub_timer_callback)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mpc_position', 100)
        self.subscription = self.create_subscription(
            Vector3,
            '/odom/data',
            self.feedback_callback,
            10)
        self.cx, self.cy, self.cyaw, self.ck = get_straight_course(0.1)

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
        self.ck_test = 0.0
        
    def feedback_callback(self, msg):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_yaw = msg.z
    def pub_timer_callback(self):
        msg = Float32MultiArray()
        if self.tick <= len(self.cx) - 1 : 
            self.pub_x = self.cx[self.tick]
            self.pub_y = self.cy[self.tick]
            self.pub_yaw = self.cyaw[self.tick]
            self.pub_yaw = 0.0
            if self.tick <= len(self.cx) - 5 :
                self.ck_test = self.ck[self.tick + 4]
        if self.tick > 5 : self.slow_speed = 0.0
        msg.data = [(float) (self.pub_x), (float) (self.pub_y), (float) (self.pub_yaw),(float) (self.slow_speed)]
        self.publisher_.publish(msg)

    def prosses_callback(self):
        self.state_init = ca.DM([self.current_x, self.current_y, self.current_yaw])
        self.state_target = ca.DM([self.pub_x, self.pub_y, self.pub_yaw])
        if( ca.norm_2(self.state_init - self.state_target) <= 2) : 
            # self.tick = self.tick + 1
            if(abs(self.ck_test)) > 0.10:
                self.tick = self.tick + 1
            else : 
                self.tick = self.tick + 2


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