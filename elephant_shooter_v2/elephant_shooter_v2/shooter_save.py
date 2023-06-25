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
range_of_shoot = 1.5 # m


def read_and_modify_one_block_of_yaml_data(filename, key, value):
    with open(f'{filename}', 'r') as f:
        data = yaml.safe_load(f)
        data[f'{key}'] = value 
        print(data) 
    with open(f'{filename}', 'w') as file:
        yaml.dump(data,file,sort_keys=False)
    print('done!')

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

        self.save_speed = 0.0
        self.save_dist = 0.0

        ## file yaml
        self.pole_1_file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_shooter_v2/elephant_shooter_v2/param/pole_1.yaml'
        self.pole_2_file = f'/home/{username}/Elephant_ws/src/Elephant_robocon2023/elephant_shooter_v2/elephant_shooter_v2/param/pole_2.yaml'
        self.i_pole_2 = 0
        self.i_pole_1 = 0
        ## push button
        self.push_shoot = 0
        self.push_save = 0
        self.saved = 0

        self.init_read_param()
    
    def init_read_param(self):
        self.pole_1_a, self.pole_1_b = read_one_block_of_yaml_data(self.pole_1_file, key = 'linear') 
        self.pole_2_a, self.pole_2_b = read_one_block_of_yaml_data(self.pole_2_file, key = 'linear') 

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
            power = self.distant + self.increase - self.decrease
            if (power < 0): 
                power = 0
                
            shoot_speed = self.speed(power)
            shoot_speed_pub.data = shoot_speed

            self.shoot_publisher_.publish(shoot_speed_pub)

            state = UInt8()
            state.data = 2
            self.state_pub.publish(state)

            self.save_speed = (float)(shoot_speed)
            self.save_dist = self.distant

        elif (joy_msg.buttons[2] == 0 and self.push_shoot == 1):
            self.push_shoot = 0

        if (joy_msg.buttons[9] == 1 and self.push_save == 0):
            self.push_save = 1
            if(self.saved == 0):
                self.saved = 1
                if (self.distant > 0.6 and self.distant < 2.3) :
                    read_and_modify_one_block_of_yaml_data(self.pole_1_file, key=f'point_{self.i_pole_1}', value = [self.save_dist,self.save_speed])
                    self.get_logger().info( "%d \t" % self.i_pole_1+'saved shoot speed to yaml !!!' ,throttle_duration_sec=print_time)
                    self.i_pole_1 = self.i_pole_1 + 1
                    read_and_modify_one_block_of_yaml_data(self.pole_1_file, key=f'len_i', value = self.i_pole_1)
                elif (self.distant > 2.3):
                    read_and_modify_one_block_of_yaml_data(self.pole_2_file, key=f'point_{self.i_pole_2}', value = [self.save_dist,self.save_speed])
                    self.get_logger().info( "%d \t" % self.i_pole_2+'saved shoot speed to yaml !!!' ,throttle_duration_sec=print_time)
                    self.i_pole_2 = self.i_pole_2 + 1
                    read_and_modify_one_block_of_yaml_data(self.pole_2_file, key=f'len_i', value = self.i_pole_2)


        
        elif (joy_msg.buttons[9] == 0 and self.push_save == 1):
            self.push_save = 0
            self.saved = 0

        


    def speed(self, x):
        # y = -14.19*np.power(x,7) + 238.3*np.power(x,6) - 1638*np.power(x,5) + 5946*np.power(x,4) - 1.226e+04*np.power(x,3) + 1.43e+04*np.power(x,2) - 8632*x + 2446
        # y = 37.72 * np.power(x,3) - 276.3 * np.power(x,2) + 578.7 * x + 13.44
        if (self.distant > 0.6 and self.distant < 1.8) :    # pole 1
            y = self.pole_1_a * x + self.pole_1_b
        elif (self.distant > 1.8 ):
            y = self.pole_2_a * x + self.pole_2_b
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