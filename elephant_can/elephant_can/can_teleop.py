import rclpy
from rclpy.node import Node
from kinematic_model.kinematic import *

# === ROS2 message ===
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import UInt8

print_time = 0.5


import yaml

import getpass
username = getpass.getuser()

def read_one_block_of_yaml_data(filename, key):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f)
    return output[f'{key}'] 

class ros_node(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.kinematic = kinematic()
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.control_type = True

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback,25)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback,10)
        self.velocity_pub = self.create_publisher(Float32MultiArray, 'pub_speed', 10)
        self.joy_pos_pub = self.create_publisher(Vector3, 'joy_position', 10)

        # self.shooter_speed_pub = self.create_publisher(Int8, 'shooter_command', 10)
        # self.state_pub = self.create_publisher(UInt8, 'process_state', 10)
        # self.adjust_up_pub = self.create_publisher(Float32, 'adjust_up', 10)
        # self.adjust_down_pub = self.create_publisher(Float32, 'adjust_down', 10)
        self.velocity_timer = self.create_timer(0.01, self.velocity_callback)
        ##
        self.control_type_pub = self.create_publisher(Bool, 'Controller_state', 10)

        # self.omega_sub = self.create_subscription(Float32,"/omega_control",self.omega_sub_cb, 10)

        self.store_speed = 0.0
        self.reload = 0

        ## data of joy stick
        self.recived = 0
        self.moving_process = 0
        self.press_button = 0
        self.count = 0
        ## point read from yaml file
        self.file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_can/elephant_can/param/moving_pos.yaml'
        self.point_1 = read_one_block_of_yaml_data(self.file, key='point_1')
        self.point_2 = read_one_block_of_yaml_data(self.file, key='point_2')
        self.point_3 = read_one_block_of_yaml_data(self.file, key='point_3')
        self.point_4 = read_one_block_of_yaml_data(self.file, key='point_4')
        self.point_5 = read_one_block_of_yaml_data(self.file, key='point_5')
        self.point_6 = read_one_block_of_yaml_data(self.file, key='point_6')
        self.point_7 = read_one_block_of_yaml_data(self.file, key='point_7')

        self.ring_R = read_one_block_of_yaml_data(self.file, key='ring_R')
        self.ring_L = read_one_block_of_yaml_data(self.file, key='ring_L')

        # self.omega = 0.0
        ##
        self.omega_gain = 0.7
        self.push_omega = 0

        self.vel_gain = 2.0
        self.push_vel = 0

    # def omega_sub_cb(self,omega_msg):
    #     self.omega = omega_msg.data

    def joy_callback(self, joy_msg):
        self.vx = joy_msg.axes[1]
        self.vy = joy_msg.axes[0]
        self.omega = -1 * joy_msg.axes[3]

        # shoot_msg = Int8()
        # shoot_msg.data = joy_msg.buttons[2]
        # self.shooter_speed_pub.publish(shoot_msg)

        # adjust_down_msg = Float32()
        # adjust_down_msg.data = float(joy_msg.axes[5])
        # self.adjust_down_pub.publish(adjust_down_msg)

        # adjust_up_msg = Float32()
        # adjust_up_msg.data = float(joy_msg.axes[2])
        # self.adjust_up_pub.publish(adjust_up_msg)

        # if (joy_msg.buttons[3] == 1):
        #     state = UInt8()
        #     state.data = 1
        #     self.state_pub.publish(state)
        
        # if (joy_msg.buttons[1] == 1):
        #     state = UInt8()
        #     state.data = 0
        #     self.state_pub.publish(state)
            

        if joy_msg.buttons[8] == 1 and joy_msg.buttons[9] == 0:
            self.control_type = True
        elif joy_msg.buttons[8] == 0 and joy_msg.buttons[9] == 1:
            self.control_type = False

        if(joy_msg.buttons[12] == 1 and self.push_omega == 0):
            self.push_omega = 1
            if(self.omega_gain == 0.7):
                self.omega_gain = 0.2
            elif(self.omega_gain == 0.2):
                self.omega_gain = 0.7
        elif(joy_msg.buttons[12] == 0 and self.push_omega == 1):
            self.push_omega = 0
        
        if(joy_msg.buttons[11] == 1 and self.push_vel == 0):
            self.push_vel = 1
            if(self.vel_gain == 2.0):
                self.vel_gain = 0.5
            elif(self.vel_gain == 0.5):
                self.vel_gain = 2.0
        elif(joy_msg.buttons[11] == 0 and self.push_vel == 1):
            self.push_vel = 0


        if self.control_type == True : 
            joy_pub_msg = Vector3()

            if (joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 0):
                if(joy_msg.axes[7] > 0.0 and self.press_button == 0):
                    self.press_button = 1
                    self.count = self.count + 1
                    if(self.count > 6) : self.count = 6
                elif ( joy_msg.axes[7] < 0.0 and self.press_button == 0):
                    self.press_button = 1
                    self.count = self.count - 1
                    if(self.count < 0) : self.count = 0
                elif (joy_msg.axes[7] == 0.0 and self.press_button == 1):
                    self.press_button = 0

                ## process pub position
                if(self.count == 0):
                    if (joy_msg.buttons[0] == 0):
                        self.get_logger().info('position to go :: point_1', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_1[0]
                        joy_pub_msg.y = self.point_1[1]
                        joy_pub_msg.z = self.point_1[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!' , throttle_duration_sec=print_time)

                elif(self.count == 1):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_2', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_2[0]
                        joy_pub_msg.y = self.point_2[1]
                        joy_pub_msg.z = self.point_2[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)

                elif(self.count == 2):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_3', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_3[0]
                        joy_pub_msg.y = self.point_3[1]
                        joy_pub_msg.z = self.point_3[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)
            
                elif(self.count == 3):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_4', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_4[0]
                        joy_pub_msg.y = self.point_4[1]
                        joy_pub_msg.z = self.point_4[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)

                elif(self.count == 4):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_5', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_5[0]
                        joy_pub_msg.y = self.point_5[1]
                        joy_pub_msg.z = self.point_5[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)

                elif(self.count == 5):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_6', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_6[0]
                        joy_pub_msg.y = self.point_6[1]
                        joy_pub_msg.z = self.point_6[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)

                elif(self.count == 6):
                    if (joy_msg.buttons[0] == 0):
                        
                        self.get_logger().info('position to go :: point_7', throttle_duration_sec=print_time)
                    else :
                        joy_pub_msg.x = self.point_7[0]
                        joy_pub_msg.y = self.point_7[1]
                        joy_pub_msg.z = self.point_7[2]

                        self.joy_pos_pub.publish(joy_pub_msg)
                        self.get_logger().info('Go!!!!!!!!!!!!!!!!!!', throttle_duration_sec=print_time)

            elif (joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 0):
                if(joy_msg.buttons[0] == 0):
                    self.get_logger().info('position to go :: Left ring zone !!!!', throttle_duration_sec=print_time)
                else :
                    joy_pub_msg.x = self.ring_L[0]
                    joy_pub_msg.y = self.ring_L[1]
                    joy_pub_msg.z = self.ring_L[2]
                    self.joy_pos_pub.publish(joy_pub_msg)
                    self.get_logger().info('Go !!!! dont forget to tape pick up', throttle_duration_sec=print_time)
            elif (joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 1):
                if(joy_msg.buttons[0] == 0):
                    self.get_logger().info('position to go :: Right ring zone !!!!', throttle_duration_sec=print_time)
                else :
                    joy_pub_msg.x = self.ring_R[0]
                    joy_pub_msg.y = self.ring_R[1]
                    joy_pub_msg.z = self.ring_R[2]
                    self.joy_pos_pub.publish(joy_pub_msg)
                    self.get_logger().info('Go !!!! dont forget to tape pick up', throttle_duration_sec=print_time)          
                        
            

    def twist_callback(self, twist_msg):
        if self.control_type == False :
            self.vx = twist_msg.linear.x
            self.vy = twist_msg.linear.y
            self.omega = twist_msg.angular.z

    def velocity_callback(self):
        if self.control_type == False :
            self.get_logger().info('manual mode!!!', throttle_duration_sec=print_time)
            pub_msg = Float32MultiArray()
            Vx = self.kinematic.map(self.vx, -1 , 1,-1 * self.vel_gain,self.vel_gain)
            Vy = self.kinematic.map(self.vy, -1, 1, -1 * self.vel_gain, self.vel_gain)
            Vth = self.kinematic.map(self.omega, -1,1,-1 * self.omega_gain, self.omega_gain)
            # Vth = self.omega
            w1,w2,w3,w4 = self.kinematic.inverse_kinematic(Vx,Vy,Vth)
            pub_msg.data = [float (w1), float (w2), float (w3), float (w4)]
            data = np.array([w1, w2, w3, w4])
            self.velocity_pub.publish(pub_msg)

        pub_type_msg = Bool()
        pub_type_msg.data = self.control_type
        self.control_type_pub.publish(pub_type_msg)

        


def main(args=None):
    rclpy.init(args=args)
    teleop_node = ros_node()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()