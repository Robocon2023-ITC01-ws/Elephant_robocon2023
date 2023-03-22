import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can
import numpy as np

wheel_d = 0.058  # in meter

def position( tick, dir, count):
    new_count = tick
    pos = 0
    # if(dir == False):
    #     diff = 65536 - new_count 
    # else:
    #     diff = new_count
    if (dir == False):
        if(new_count >= count):
            diff = new_count - count
        else:
            diff = (65536 - new_count) + count
        pos = -np.pi * wheel_d * diff/8195   

    else:
        if(new_count<=count):
            diff = count - new_count
        else:
            diff = (65536 - count) + new_count
        pos = np.pi * wheel_d * diff/8195

    count = new_count
    return pos

class ros_node(Node):
    def __init__(self):
        super(ros_node, self).__init__('odom_node')
        self.timer = 0.01
        self.bus = can.interface.Bus(channel='can0', interface = 'socketcan', bitrate = 1000000)

        self.encoder_tick = np.zeros(3)
        self.encoder_dir = np.zeros(3)
        self.count = 0
        self.X = 0.0 # in meter
        self.Y = 0.0 # in meter
        self.theta = 0.0 # in meter
        

        self.odom_pub = self.create_publisher(Float32MultiArray, 'odom', 10)
        #self.odom_timer = self.create_timer(self.timer, self.odom_callback)
        while(rclpy.ok()):
            msg = self.bus.recv(0.01)
            self.tick = (msg.data[0] << 8) + msg.data[1]
            # encoder_tick[1] = (msg.data[2] << 8) + msg.data[3]
            # encoder_tick[2] = (msg.data[4] << 8) + msg.data[5]

            dir = (msg.data[6] & 0x01) == 0x01
            self.X = position(self.tick, dir, self.count)
            pos_msg = Float32MultiArray()
            pos_msg.data = [self.X]
            print(self.X)
            
            self.odom_pub.publish(pos_msg)
        
    
        


def main(args=None):
    rclpy.init(args=args)
    odom_node = ros_node()
    rclpy.spin(odom_node)
    odom_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()

