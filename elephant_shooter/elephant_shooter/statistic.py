import rclpy 
import numpy as np
from rclpy.node import Node
import yaml

from std_msgs.msg import UInt16, UInt8
from std_msgs.msg import Int8, Float32, Float32MultiArray


class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.adjust_up_sub = self.create_subscription(Float32, "adjust_down", self.adjust_up_callback,10)
        self.adjust_down_sub = self.create_subscription(Float32, "adjust_up", self.adjust_down_callback, 10)
        self.shooter_pub = self.create_publisher(UInt16, 'shooter', 10)
        self.param_pub = self.create_publisher(Float32MultiArray, 'stored', 10)
        self.shoot_pub = self.create_publisher(UInt8, 'process_state', 10)

        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.adjust = 0.0
        self.xin = 0.0
        self.adjust_up = 0
        self.adjust_down = 0
        self.speed1 = 0.0

    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value
    
    def speed(self, x):
        y = -14.19*np.power(x,7) + 238.3*np.power(x,6) - 1638*np.power(x,5) + 5946*np.power(x,4) - 1.226e+04*np.power(x,3) + 1.43e+04*np.power(x,2) - 8632*x + 2446
        return y

    def adjust_up_callback(self, adjust_msg):
        self.adjust_up = adjust_msg.data

    def adjust_down_callback(self, adjust_msg):
        self.adjust_down = adjust_msg.data

    def laser_callback(self, laser_msg):
        self.laser_data = laser_msg.data


    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
        
        while(button_command == 1): 
            self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
            distance = (4.439 - 0.765)/(3495 - 6)*(self.laser_data - 6) + 0.765
            shooter_speed = self.speed(distance)
            if(self.adjust_up != 1 and self.adjust_down == 1):
                self.speed1 = self.map(self.adjust_up, -1.0, 1.0, shooter_speed+25, shooter_speed)
            elif(self.adjust_up == 1 and self.adjust_down != 1):
                self.speed1 = self.map(self.adjust_down, -1.0, 1.0, shooter_speed-25, shooter_speed)
            else:
                self.speed1 = shooter_speed
            print(distance, self.speed1)
            self.rps = int(self.map(self.speed1, 0, 1500, 0, 65535))
            shooter_msg = UInt16()
            shooter_msg.data = self.rps
            self.shooter_pub.publish(shooter_msg)
            ## here
            shoot_msg = UInt8()
            shoot_msg.data = 2
            self.shoot_pub.publish(shoot_msg)
            self.rps = 0
            break


    



def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()