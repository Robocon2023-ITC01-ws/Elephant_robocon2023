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

import getpass
username = getpass.getuser()

print_time = 0.5

import yaml
def read_and_modify_one_block_of_yaml_data(filename, key, value):
    with open(f'{filename}', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = value 
        print(data) 
    with open(f'{filename}', 'w') as file:
        yaml.dump(data,file,sort_keys=False)
    print('done!')
    print('done!')

gain = 2
omega_gain = 0.5

class ros_node(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.kinematic = kinematic()
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.control_type = False

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback,20)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback,10)
        self.save_pos_sub = self.create_subscription(Vector3, '/odom/data', self.save_pos_cb ,10)
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

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_yaw = 0.0

        self.press_button = 0
        self.count = 0

        self.file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_can/elephant_can/param/moving_pos.yaml'

    def save_pos_cb(self, pos_msg):
        self.pos_x = pos_msg.x
        self.pos_y = pos_msg.y
        self.pos_yaw = pos_msg.z

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
        if self.control_type == False : 

            self.get_logger().info("%f\t" % self.pos_x + "%f\t" % self.pos_y +"%f" % self.pos_yaw , throttle_duration_sec = print_time)
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

            if(joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 0 and joy_msg.axes[6] == 0.0):
                if(self.count == 0):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_1', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_1' ,throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 1' ,throttle_duration_sec=print_time)

                elif(self.count == 1):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_2', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_2', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 2' , throttle_duration_sec=print_time)

                elif(self.count == 2):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_3', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_3', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 3', throttle_duration_sec=print_time)
            
                elif(self.count == 3):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_4', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_4', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 4', throttle_duration_sec=print_time)

                elif(self.count == 4):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_5', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_5', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 5', throttle_duration_sec=print_time)

                elif(self.count == 5):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_6', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_6', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 6' ,throttle_duration_sec=print_time)

                elif(self.count == 6):
                    if (joy_msg.buttons[0] == 1):
                        read_and_modify_one_block_of_yaml_data(self.file, key='point_7', value = [self.pos_x, self.pos_y, self.pos_yaw])
                        self.get_logger().info('saved !! to point_7', throttle_duration_sec=print_time)
                    else :
                        self.get_logger().info('point 7', throttle_duration_sec=print_time)
            elif (joy_msg.buttons[4] == 1 and joy_msg.buttons[5] == 0):
                if(joy_msg.buttons[0] == 1):
                    read_and_modify_one_block_of_yaml_data(self.file, key='ring_L', value = [self.pos_x, self.pos_y, self.pos_yaw])
                    self.get_logger().info('save!!! Left ring zone !!!!', throttle_duration_sec=print_time)
                else :
                    self.get_logger().info('Left ring zone', throttle_duration_sec=print_time)
            elif (joy_msg.buttons[4] == 0 and joy_msg.buttons[5] == 1):
                if(joy_msg.buttons[0] == 1):
                    read_and_modify_one_block_of_yaml_data(self.file, key='ring_R', value = [self.pos_x, self.pos_y, self.pos_yaw])
                    self.get_logger().info('save!!! Right ring zone !!!!', throttle_duration_sec=print_time)
                else :
                    self.get_logger().info('Right ring zone', throttle_duration_sec=print_time)
            elif (joy_msg.axes[6] > 0.0):
                if(joy_msg.buttons[0] == 1):
                    read_and_modify_one_block_of_yaml_data(self.file, key='limit_L', value = [self.pos_x, self.pos_y, self.pos_yaw])
                    self.get_logger().info('save!!! left limit point !!!!', throttle_duration_sec=print_time)
                else :
                    self.get_logger().info('Left limit point', throttle_duration_sec=print_time)
            elif (joy_msg.axes[6] < 0.0):
                if(joy_msg.buttons[0] == 1):
                    read_and_modify_one_block_of_yaml_data(self.file, key='limit_R', value = [self.pos_x, self.pos_y, self.pos_yaw])
                    self.get_logger().info('save!!! Right limit point !!!!', throttle_duration_sec=print_time)
                else :
                    self.get_logger().info('Right limit point', throttle_duration_sec=print_time)

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
            Vth = self.kinematic.map(self.omega, -1,1,-1 * omega_gain, omega_gain)
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