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
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import UInt8

gain = 2
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

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback,20)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback,10)
        self.velocity_pub = self.create_publisher(Float32MultiArray, 'pub_speed', 10)
        self.joy_pos_pub = self.create_publisher(Vector3, 'joy_position', 10)

        self.shooter_speed_pub = self.create_publisher(Int8, 'shooter_command', 10)
        self.state_pub = self.create_publisher(UInt8, 'process_state', 10)
        self.adjust_pub = self.create_publisher(Float32, 'adjust', 10)
        self.velocity_timer = self.create_timer(0.01, self.velocity_callback)
        ##
        self.control_type_pub = self.create_publisher(Bool, 'Controller_state', 10)

        self.store_speed = 0.0
        self.reload = 0

        ## data of joy stick
        self.recived = 0
        self.moving_process = 0
        ## point read from yaml file
        self.file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_can/elephant_can/param/moving_pos.yaml'
        self.point_1 = read_one_block_of_yaml_data(self.file, key='point_1')
        self.point_2 = read_one_block_of_yaml_data(self.file, key='point_2')
        self.point_3 = read_one_block_of_yaml_data(self.file, key='point_3')
        self.point_4 = read_one_block_of_yaml_data(self.file, key='point_4')
        self.point_5 = read_one_block_of_yaml_data(self.file, key='point_5')
        self.point_6 = read_one_block_of_yaml_data(self.file, key='point_6')
        self.point_7 = read_one_block_of_yaml_data(self.file, key='point_7')
        self.point_1 = read_one_block_of_yaml_data(self.file, key='point_1')
        self.ring_R = read_one_block_of_yaml_data(self.file, key='ring_R')
        self.ring_L = read_one_block_of_yaml_data(self.file, key='ring_L')

    def joy_callback(self, joy_msg):
        self.vx = joy_msg.axes[1]
        self.vy = joy_msg.axes[0]
        self.omega = -1 * joy_msg.axes[3]

        shoot_msg = Int8()
        shoot_msg.data = joy_msg.buttons[2]
        self.shooter_speed_pub.publish(shoot_msg)

        adjust_msg = Float32()
        adjust_msg.data = float(joy_msg.axes[5])
        self.adjust_pub.publish(adjust_msg)

        if (joy_msg.buttons[3] == 1):
            state = UInt8()
            state.data = 1
            self.state_pub.publish(state)
        
        if (joy_msg.buttons[1] == 1):
            state = UInt8()
            state.data = 0
            self.state_pub.publish(state)
            

        if joy_msg.buttons[8] == 1 and joy_msg.buttons[9] == 0:
            self.control_type = True
        elif joy_msg.buttons[8] == 0 and joy_msg.buttons[9] == 1:
            self.control_type = False


        if self.control_type == True : 
            if joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 0:
                msg = Vector3()
                msg.x = 4.2        ##  
                msg.y = -5.6
                msg.z = 0.0
                self.joy_pos_pub.publish(msg)
            elif joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 1:
                msg = Vector3()
                msg.x = 3.6
                msg.y = 4.9
                msg.z = 0.0
                self.joy_pos_pub.publish(msg)
            
            
            # if joy_msg.buttons[11] == 1 and self.recived == 0:   ## left ring
            #     self.recived = 1
            
            # if joy_msg.buttons[12] == 1 and self.recived == 0 : ## right ring
            #     self.recived = 1
            
            # if joy_msg.axes[7] > 0.0 and self.recived == 0:
            #     self.recived = 1
            #     self.moving_process = self.moving_process + 1

            # elif joy_msg.axes[7] < 0.0 and self.recived == 0:
            #     self.recived = 1
            #     self.moving_process = self.moving_process - 1

            # elif joy_msg.axes[7] == 0.0 and self.recived == 1:
            #     self.recived = 0

            self.get_logger().info("%d" % self.moving_process)
            



    def twist_callback(self, twist_msg):
        if self.control_type == False :
            self.vx = twist_msg.linear.x
            self.vy = twist_msg.linear.y
            self.omega = twist_msg.angular.z

    def velocity_callback(self):
        if self.control_type == False :
            pub_msg = Float32MultiArray()
            Vx = self.kinematic.map(self.vx, -1 , 1,-1 * gain,gain)
            Vy = self.kinematic.map(self.vy, -1, 1, -1 * gain, gain)
            Vth = self.kinematic.map(self.omega, -1,1,-1, 1)
            w1,w2,w3,w4 = self.kinematic.inverse_kinematic(Vx,Vy,Vth)
            pub_msg.data = [float (w1), float (w2), float (w3), float (w4)]
            data = np.array([w1, w2, w3, w4])
            print(Vx)
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