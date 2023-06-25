import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy

import yaml
import numpy as np
import getpass
username = getpass.getuser()
print_time = 0.5
###
range_of_shoot = 0.5 # m

def read_and_modify_one_block_of_yaml_data(filename, key, value):
    with open(f'{filename}', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = value 
        # print(data) 
    with open(f'{filename}', 'w') as file:
        yaml.dump(data,file,sort_keys=False)
    print('done!!!!!!')

def read_one_block_of_yaml_data(filename, key):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f)
    return output[f'{key}'] 


class shooter(Node):
    def __init__(self):
        super().__init__('shooter_test')
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback,10)
        self.laser_sub = self.create_subscription(Float32, 'laser_distant', self.laser_dist_callback, 10)
        self.shoot_publisher_ = self.create_publisher(Float32, 'shooter_speed', 10)
        self.state_pub = self.create_publisher(
                UInt8,
                'process_state',
                10)
        self.state_pub = self.create_publisher(UInt8, 'process_state', 10)

        self.distant = 0.0
        self.increase = 0.0
        self.decrease = 0.0

        ## file yaml
        ## file yaml
        self.pole_1_file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_shooter_v2/elephant_shooter_v2/param/pole_1.yaml'
        self.pole_2_file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_shooter_v2/elephant_shooter_v2/param/pole_2.yaml'
                ## push button
        self.push_shoot = 0
        self.push_save = 0
        self.saved = 0

        self.init_model_from_param()

    def init_model_from_param(self):
        pole_1_len = read_one_block_of_yaml_data(self.pole_1_file, key = 'len_i') 
        pole_2_len = read_one_block_of_yaml_data(self.pole_2_file,key = 'len_i') 

        pole_1_distant = np.zeros(pole_1_len)
        pole_1_speed = np.zeros(pole_1_len)

        pole_2_distant = np.zeros(pole_2_len)
        pole_2_speed = np.zeros(pole_2_len)

        for i in range(pole_1_len):
            data = read_one_block_of_yaml_data(self.pole_1_file, key = f'point_{i}')
            pole_1_distant[i] = data[0]
            pole_1_speed[i] = data[1]

        z = np.polyfit(pole_1_distant, pole_1_speed, 1)
        read_and_modify_one_block_of_yaml_data(self.pole_1_file, key = 'linear', value = [(float)(z[0]), (float)(z[1])])
        self.pole_1_power = np.poly1d(z)

        for i in range (pole_2_len):
            data = read_one_block_of_yaml_data(self.pole_2_file, key = f'point_{i}')
            pole_2_distant[i] = data[0]
            pole_2_speed[i] = data[1]
        z2 = np.polyfit(pole_2_distant, pole_2_speed,1)
        read_and_modify_one_block_of_yaml_data(self.pole_2_file, key = 'linear', value = [(float)(z2[0]),(float)(z2[1])])
        self.pole_2_power = np.poly1d(z2)


    def laser_dist_callback(self, dist_msg):
        self.distant = dist_msg.data
        
    def joy_callback(self,joy_msg):
        self.increase = self.map(joy_msg.axes[5], 1.0, -1.0 , 0, range_of_shoot)
        self.decrease = self.map(joy_msg.axes[2], 1.0, -1.0 , 0, range_of_shoot)


        if (joy_msg.buttons[3] == 1):
            state = UInt8()
            state.data = 1
            self.state_pub.publish(state)
        
        if (joy_msg.buttons[1] == 1):
            state = UInt8()
            state.data = 0
            self.state_pub.publish(state)

        if (joy_msg.buttons[2] == 1 and self.push_shoot == 0):
            self.push_shoot = 1

            shoot_speed_pub = Float32()
            shoot_speed = self.speed(self.distant + self.increase - self.decrease)
            shoot_speed_pub.data = shoot_speed

            self.shoot_publisher_.publish(shoot_speed_pub)

            state = UInt8()
            state.data = 2
            self.state_pub.publish(state)

            self.save_speed = (float)(shoot_speed)
            self.save_dist = self.distant

        elif (joy_msg.buttons[2] == 0 and self.push_shoot == 1):
            self.push_shoot = 0

    
    def speed(self, x):
        if (self.distant > 0.6 and self.distant < 2.3) :    # pole 1
            y = self.pole_1_power(x)
        elif (self.distant > 2.3 ):
            y = self.pole_2_power(x)
        return y
    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value

    


def main(args=None):
    rclpy.init(args=args)
    shooter_node = shooter()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
