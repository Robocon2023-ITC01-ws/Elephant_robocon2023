import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import UInt16 # laser: subscribe the data from laser
from std_msgs.msg import UInt8 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from std_msgs.msg import Float32
from time import sleep

from shooter.ER_shooter import *

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        #self.laser_sub = self.create_subscription(Int16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.adjust_sub = self.create_subscription(Float32, "adjust", self.adjust_callback,10)
        self.shooter_pub = self.create_publisher(UInt16, 'shooter', 10)
        self.shoot_pub = self.create_publisher(UInt8, 'process_state', 10)

        self.button_command = 0
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0
        self.adjust = 0.0
    
    def adjust_callback(self, adjust_msg):
        self.adjust = adjust_msg.data

    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
        while(button_command == 1):
            self.laser_sub = self.create_subscription(UInt16, 'laser', self.laser_callback, 10)
            distance = (4.439 - 0.765)/(3495 - 6)*(self.laser_data - 6) + 0.765
            print(distance)
            self.rps = int(shooter(distance, self.adjust).shooter())
            if(self.rps == 7433):
                self.rps = 0 
            shooter_msg = UInt16()
            shooter_msg.data = self.rps
            self.shooter_pub.publish(shooter_msg)
            ## here
            shoot_msg = UInt8()
            shoot_msg.data = 2
            self.shoot_pub.publish(shoot_msg)
            self.rps = 0
            break

        

    def laser_callback(self, laser_msg):
        self.laser_data = laser_msg.data
        

def main(args=None):
    rclpy.init(args=args)
    shooter_node = ShooterNode()
    rclpy.spin(shooter_node)
    shooter_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()

