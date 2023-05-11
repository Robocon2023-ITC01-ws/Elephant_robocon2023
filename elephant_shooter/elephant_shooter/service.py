from example_interfaces.srv import SetBool

import rclpy
import string
from rclpy.node import Node
from std_msgs.msg import Int32, Int16, String
import time

from shooter.ER_shooter import *

def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value

class ShooterService(Node):
    def __init__(self):
        super().__init__('shooter_service')
        self.shooter_pub = self.create_publisher(Int32, 'shooter', 10)
        self.laser_sub = self.create_subscription(Int16, 'laser', self.laser_callback, 10)
        self.shooter_srv = self.create_service(SetBool, 'shooter_srv', self.shooter_srv_callback)

        # Timer
        self.timer = 0.01
        # self.shooter_timer = self.create_timer(self.timer, self.shooter_callback)
        self.rps = 0
        # Parameter
        self.data = 0
        self.shoot_data = 0
        self.distance = 0.0
        self.stored = 0
        self.flag = False
        self.commant = False

    def shooter_srv_callback(self, request, response):
        self.commant = bool(request.data)
        response.success= False
        response.message = "False"
        while(self.commant == True):
            
            response.success = True
            response.message = "Success"
            print(self.data)
            self.distance = 0.001557156*(self.data) + 0.1372142
            print(self.distance)
            self.rps = int(shooter(self.distance).shooter())
            shooter_msg = Int32()
            shooter_msg.data = -self.rps
            self.shooter_pub.publish(shooter_msg)
            time.sleep(2)
            shooter_msg.data = self.rps
            self.shooter_pub.publish(shooter_msg)
            time.sleep(1)
            shooter_msg.data = 0
            self.shooter_pub.publish(shooter_msg)

            break
        return response

    def laser_callback(self, laser_msg):
        self.data = laser_msg.data
           
            
def main(args=None):
    rclpy.init(args=args)
    shooter_service = ShooterService()
    rclpy.spin(shooter_service)
    shooter_service.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
