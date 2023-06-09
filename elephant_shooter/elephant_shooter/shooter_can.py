import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16

import can
import numpy as np
import time

def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value

class can_node(Node):
    def __init__(self):
        super().__init__('can_node')
        self.bus = can.interface.Bus(channel='can0', interface='socketcan',bitrate=1000000)

        # Publish subscriber
        self.laser_pub = self.create_publisher(UInt16, 'laser', 10)
        self.shooter_sub = self.create_subscription(UInt16, 'shooter', self.shooter_callback, 10)
        self.laser_timer = self.create_timer(0.01, self.laser_callback)

        self.data = 0
        self.shooter_data = 0
        self.TxData0 = [0, 0, 0, 0, 0, 0, 0, 0]
        self.TxData1 = [0, 0, 0]
        self.encoder = 0

    def shooter_callback(self, shooter_msg):
        self.shooter_data = shooter_msg.data
        self.TxData1[0] = (self.shooter_data & 0xFF00) >> 8
        self.TxData1[1] = self.shooter_data & 0x00FF
        msg = can.Message(arbitration_id= 0x222, data= self.TxData1, dlc=3, is_extended_id=False)
        self.bus.send(msg, 0.01)
        time.sleep(2)
        if(self.shooter_data > 0):
            self.TxData1[2] = 1
            msg = can.Message(arbitration_id= 0x222, data= self.TxData1, dlc=3, is_extended_id=False)
            self.bus.send(msg, 0.01)
            time.sleep(0.2)
            self.TxData1[2] = 0
            msg = can.Message(arbitration_id= 0x222, data= self.TxData1, dlc=3, is_extended_id=False)
            self.bus.send(msg, 0.01)
        else:
            self.TxData1[2] = 0
            msg = can.Message(arbitration_id= 0x222, data= self.TxData1, dlc=3, is_extended_id=False)
            self.bus.send(msg, 0.01)

    def laser_callback(self):
        laser_msg = Int16()
        self.TxData0[0] = (self.encoder & 0xFF00) >> 8
        self.TxData0[1] = self.encoder & 0x00FF
        self.TxData0[2] = (self.encoder & 0xFF00) >> 8
        self.TxData0[3] = self.encoder & 0x00FF
        self.TxData0[4] = (self.encoder & 0xFF00) >> 8
        self.TxData0[5] = self.encoder & 0x00FF
        self.TxData0[6] = (self.encoder & 0xFF00) >> 8
        self.TxData0[7] = self.encoder & 0x00FF
        msg_ = can.Message(arbitration_id = 0x111, data= self.TxData0, dlc = 8, is_extended_id = False)
        self.bus.send(msg_, 0.01)
        can_msg = self.bus.recv(0.01)
        if(can_msg != None):
            if can_msg.arbitration_id_id == 0x333:
                self.data = (can_msg.data[6] << 8) + can_msg.data[7]
                laser_msg.data = int(self.data)
            else:
                self.data = 0
        self.laser_pub.publish(laser_msg)

        
def main(args=None):
    rclpy.init(args=args)

    can_class_test = can_node()

    rclpy.spin(can_class_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_class_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
          