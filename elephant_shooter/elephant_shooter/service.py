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
        self.laser_sub = self.create_subscription(Int16, 'laser', self.laser_callback, 10)
        self.shooter_pub = self.create_publisher(Int32, 'shooter', 10)
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
        print(self.commant)
        if(self.commant == True):
            response.success = True
            self.flag = response.success
            response.message = "Success"
        else:
            self.flag = False
            print("Error")
        return response

    def laser_callback(self, laser_msg):
        shooter_msg = Int32()
        i = 0
        if(self.flag == True):
            self.data = laser_msg.data
            while(i <=6):
                self.stored = self.data
                i = i + 1
                break
            self.distance = 0.001557156*(self.stored) + 0.1372142 
            #self.distance = map(self.data,0,4096,0.5,10.0) 
            print(self.distance)
            self.rps = shooter(self.distance).shooter()
            shooter_msg.data = int(self.rps)
            self.shooter_pub.publish(shooter_msg)

        else:
            self.distance = 0.0
            self.rps = 0
            shooter_msg.data = 0
            self.shooter_pub.publish(shooter_msg)



def main(args=None):
    rclpy.init(args=args)
    shooter_service = ShooterService()
    rclpy.spin(shooter_service)
    shooter_service.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()

