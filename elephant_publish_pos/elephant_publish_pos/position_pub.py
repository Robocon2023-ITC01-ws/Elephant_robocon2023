import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
    
try :
    from elephant_publish_pos import cubic_spline_planner
except :
    pass
try :
    import cubic_spline_planner
except :
    pass
import casadi as ca

##########################
import yaml

import getpass
username = getpass.getuser()

def read_one_block_of_yaml_data(filename, key):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f)
    return output[f'{key}'] 

file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_can/elephant_can/param/moving_pos.yaml'
Left = read_one_block_of_yaml_data(file , key = 'limit_L')
Right = read_one_block_of_yaml_data(file , key = 'limit_R')

x_left = Left[0]	
y_left = Left[1]		
x_right = Right[0]
y_right = Right[1]

distant = 0.8

# trajactory
def two_point_trajectory(dl,x0,y0,x1,y1):      # use for two points
    ax = [x0, x1]
    ay = [y0, y1]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
###################################
def three_point_trajectory(dl,x0,y0,x1,y1,x2,y2):     # three points
    ax = [x0 , x1]
    ay = [y0, y1]
    ax2 = [x1, x2]
    ay2 = [y1, y2]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax2, ay2, ds=dl)
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)
    return cx, cy, cyaw, ck

def four_point_trajectory(dl,x0,y0,x1,y1,x2,y2,x3,y3):     # four points
    ax = [x0 , x1]
    ay = [y0, y1]
    ax2 = [x1, x2]
    ay2 = [y1, y2]
    ax3 = [x2, x3]
    ay3 = [y2, y3]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax2, ay2, ds=dl)
    cx3, cy3, cyaw3, ck3, s3 = cubic_spline_planner.calc_spline_course(
        ax3, ay3, ds=dl)
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    cx.extend(cx3)
    cy.extend(cy3)
    cyaw.extend(cyaw3)
    ck.extend(ck3)
    return cx, cy, cyaw, ck

class position_class(Node):
    def __init__(self):
        super().__init__('pub_node_test')
        timer_period = 0.05 # seconds
        self.prosses = self.create_timer(timer_period, self.prosses_callback)
        self.pub_timer = self.create_timer(0.05, self.pub_timer_callback)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'mpc_position', 10)
        self.control_type_pub = self.create_subscription(Bool, 'Controller_state',self.control_type_cb, 10)
        self.subscription1 = self.create_subscription(
            Vector3,
            '/odom/data',
            self.feedback_callback,
            10)
        self.subscription2 = self.create_subscription(
            Vector3,
            'joy_position',
            self.joy_position_cb,
            10)

        ############
        self.tick = 0
        self.pub_x = 0.0
        self.pub_y = 0.0
        self.pub_yaw = 0.0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0
        self.slow_speed = 1.0
        ############
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.ck_test = 0.0
        self.run = False
        self.mpc_control = True

    def control_type_cb(self, msg):
        self.mpc_control = msg.data
        if(self.mpc_control == False) :
            self.run = False
            self.tick = 0

    def joy_position_cb(self, joy_msg):
        print("recieve")
        if (joy_msg.x != self.pos_x or joy_msg.y != self.pos_y or joy_msg.z != self.pos_yaw):
            self.pos_y = joy_msg.y      ## i decide to use normal plane here for easy to analyse position and conditional
            self.pos_x = joy_msg.x
            self.pos_yaw = joy_msg.z
            ## cubic cubic_spline_planner ## here
            if (self.current_x < x_left and self.current_y > y_left):   ## surface 1
                if(self.pos_x < x_left) :
                    ## two point 
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_y < y_left and self. pos_x > x_left):
                    ## three point
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_left - y_left/2,y_left/2
                                           ,self.pos_x,self.pos_y)
                elif(self.pos_y > y_right and self.pos_x > x_right):
                    ## four point
                    self.cx, self.cy, self.cyaw, self.ck = four_point_trajectory(0.1,self.current_x,self.current_y,x_left - y_left/2,y_left/2
                                          ,x_right + y_right/2,y_right/2,self.pos_x,self.pos_y)
            elif(self.current_x < x_left and self.current_y < y_left):  ## surface 2
                if(self.pos_x < x_left):
                    ## two point 
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_y < y_left):
                    ## two point 
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_x > x_right and self.pos_y > y_right):
                    ## three point 
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_right + y_right/2,y_right/2
                                           ,self.pos_x,self.pos_y)
            elif(self.current_x > x_left and self.current_x < x_right and self.current_y < y_left): ## surface 3
                if (self.pos_y < y_left):
                    # two points
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif (self.pos_x < x_left and self.pos_y > y_left):
                    # thee point
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_left - y_left/2,y_left/2
                                           ,self.pos_x,self.pos_y)
                elif(self.pos_x > x_right and self.pos_y > y_right):
                    # thee point
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_right + y_right/2,y_right/2
                                           ,self.pos_x,self.pos_y)
            elif(self.current_x > x_right and self.current_y < y_right):    ## surface 4
                if(self.pos_y < y_right):
                    # two points
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_x > x_right):
                    # two points
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_x < x_left and self.pos_y > y_right):
                    ## three point
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_left - y_left/2,y_left/2
                                           ,self.pos_x,self.pos_y)
            elif(self.current_x > x_right and self.current_y > y_right):    ## surface 5
                if(self.pos_x >  x_right):
                    ## two point 
                    self.cx, self.cy, self.cyaw, self.ck = two_point_trajectory(0.1,self.current_x,self.current_y,self.pos_x,self.pos_y)
                elif(self.pos_y < y_right and self. pos_x < x_right):
                    # thee point
                    self.cx, self.cy, self.cyaw, self.ck = three_point_trajectory(0.1,self.current_x,self.current_y,x_right + y_right/2,y_right/2
                                           ,self.pos_x,self.pos_y)
                elif(self.pos_y > y_right and self. pos_x < x_left):
                    ## four point
                    self.cx, self.cy, self.cyaw, self.ck = four_point_trajectory(0.1,self.current_x,self.current_y,x_right + y_right/2,y_right/2
                                          ,x_left - y_left/2,y_left/2,self.pos_x,self.pos_y)
            self.slow_speed = 1.0
            self.run = True     ## state for publish mpc_position
    def feedback_callback(self, msg):
        self.current_x = msg.x     ## different plane
        self.current_y = msg.y
        self.current_yaw = msg.z
    def pub_timer_callback(self):
        if (self.run == True):
            msg = Float32MultiArray()
            if self.tick <= len(self.cx) - 1 : 
                # self.pub_x = self.cx[self.tick]     ## different plan
                # self.pub_y = self.cy[self.tick]
                self.pub_x = self.cx[self.tick]     ## different plan
                self.pub_y = self.cy[self.tick]
                self.pub_yaw = self.cyaw[self.tick]
                self.pub_yaw = 0.0      ## here what i need to think
                if self.tick <= len(self.cx) - 5 :
                    self.ck_test = self.ck[self.tick + 4]
            else :
                self.run = False
                self.tick = 0
            if self.tick > 5 : self.slow_speed = 0.0
            msg.data = [(float) (self.pub_x), (float) (self.pub_y), (float) (self.pub_yaw),(float) (self.slow_speed)]
            self.publisher_.publish(msg)

    def prosses_callback(self):
        print(self.tick)
        self.state_init = ca.DM([self.current_x, self.current_y, self.current_yaw])
        self.state_target = ca.DM([-1 * self.pub_y, self.pub_x, self.pub_yaw])      ## wrong plane
        print(self.state_init)
        print(self.state_target)
        if self.run == True :
            ###
            if( ca.norm_2(self.state_init - self.state_target) <= 1.5) : 
                # self.tick = self.tick + 1
                if(abs(self.ck_test)) > 0.10:
                    self.tick = self.tick + 1
                else : 
                    self.tick = self.tick + 1


def main(args=None):
    rclpy.init(args=args)

    pub_node_test = position_class()

    rclpy.spin(pub_node_test) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pub_node_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
