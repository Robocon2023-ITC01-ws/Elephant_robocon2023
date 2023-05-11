import rclpy
import numpy as np
from rclpy.node import Node

from std_msgs.msg import Int16 # laser: subscribe the data from laser
from std_msgs.msg import Int32 # motor: publish the data to the motor 
from std_msgs.msg import Int8 # button: to control shoot or not shoot
from time import sleep

from shooter.ER_shooter import *

class ShooterNode(Node):
    def __init__(self):
        super().__init__('shooter_node')

        #self.laser_sub = self.create_subscription(Int16, 'laser', self.laser_callback, 10)
        self.button_sub = self.create_subscription(Int8, "shooter_command", self.button_callback, 10)
        self.shooter_pub = self.create_publisher(Int32, 'shooter', 10)

        self.button_command = False
        self.laser_data = 0
        self.shooter_data = 0
        self.distance = 0.0

    def button_callback(self, button_msg):
        button_command = int(button_msg.data)
        while(button_command == 1):
            self.laser_sub = self.create_subscription(Int16, 'laser', self.laser_callback, 10)
            distance = 0.001557156*(self.laser_data) + 0.1372142
            self.rps = int(shooter(distance).shooter())
            shooter_msg = Int32()
            shooter_msg.data = -self.rps
            self.shooter_pub.publish(shooter_msg)
            sleep(1)
            shooter_msg.data = self.rps
            self.shooter_pub.publish(shooter_msg)
            sleep(1)
            shooter_msg.data = 0
            self.shooter_pub.publish(shooter_msg)
            self.rps = 0
            break
        print(self.rps)
        

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

