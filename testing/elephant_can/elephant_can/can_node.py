import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Float32
import can 
import time
############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value

class can_class(Node):
    def __init__(self):
        super().__init__('can_node_test')
        self.bus = can.interface.Bus(channel='can0', interface='socketcan',bitrate=1000000)
        self.publisher_ = self.create_publisher(UInt16MultiArray, 'tick_feedback', 20)
        self.laser_publisher_ = self.create_publisher(Int16, 'laser', 10)
        self.can_timer = self.create_timer(0.001, self.can_callback)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pub_speed',
            self.listener_callback,
            10)
        self.shooter_subscription = self.create_subscription(
            Int32,
            'shooter',
            self.sh_listener_callback,
            10)
        self.TxData = [128,0,128,0,128,0,128,0]
        self.TxData2 = [0,0,0]
        
        self.Tick = [0,0,0,0]
        self.yaw = 0
        self.pitch = 0
        self.ax = 0
        self.ay = 0
        ############
        self.pub_shoot_speed = 0
        
    def sh_listener_callback(self, shouter_msg):
        if (shouter_msg.data > 10):
            self.TxData2[2] = 1
     
            shoot_speed = shouter_msg.data
        elif (shouter_msg.data < -10):
            self.TxData2[2] = 0
            
            shoot_speed = shouter_msg.data
        else :
            self.TxData2[2] = 0
            shoot_speed = 0
        
        self.TxData2[0] = ((shoot_speed & 0xFF00) >> 8)
        self.TxData2[1] = (shoot_speed & 0x00FF)
        self.shoot_msg = can.Message(arbitration_id=0x222,
                data=self.TxData2, dlc=3, 
                is_extended_id=False)
        self.pub_shoot_speed = 1

    def listener_callback(self, msg):
            V1_out  = (int)( map(msg.data[0], -100, 100, 0, 65535))
            V2_out  = (int)( map(msg.data[1], -100, 100, 0, 65535))
            V3_out  = (int)( map(msg.data[2], -100, 100, 0, 65535))
            V4_out  = (int)( map(msg.data[3], -100, 100, 0, 65535))
            self.TxData[0] = ((V1_out & 0xFF00) >> 8)
            self.TxData[1] = (V1_out & 0x00FF)
            self.TxData[2] = ((V2_out & 0xFF00) >> 8)
            self.TxData[3] = (V2_out & 0x00FF)
            self.TxData[4] = ((V3_out & 0xFF00) >> 8)
            self.TxData[5] = (V3_out & 0x00FF)
            self.TxData[6] = ((V4_out & 0xFF00) >> 8)
            self.TxData[7] = (V4_out & 0x00FF)
    def can_callback(self):
        pub_msg = UInt16MultiArray()
        laser_msg = Int16()
        msg = can.Message(arbitration_id=0x111,
                data=self.TxData,
                is_extended_id=False)
        try :
            if (self.pub_shoot_speed):
                self.pub_shoot_speed = 0
                self.bus.send(self.shoot_msg,0.01)
            self.bus.send(msg,0.01)
            finish_recv = True
        except can.CanError :
            pass
        while(finish_recv):  ## just for test
            try :
                can_msg = self.bus.recv(0.01)
                if(can_msg != None):
                    if can_msg.arbitration_id == 0x155:
                        self.Tick[0] = can_msg.data[0] << 8 | can_msg.data[1]
                        self.Tick[1] = can_msg.data[2] << 8 | can_msg.data[3]
                    elif can_msg.arbitration_id == 0x140:
                        finish_recv = False
                        self.Tick[2] = can_msg.data[0] << 8 | can_msg.data[1]
                        self.Tick[3] = can_msg.data[2] << 8 | can_msg.data[3]
                        pub_msg.data = [(self.Tick[0]), (self.Tick[1]), (self.Tick[2]),self.Tick[3]]         
                        self.publisher_.publish(pub_msg)
                    elif can_msg.arbitration_id == 0x333:
                        finish_recv = False
                        self.Tick[0] = can_msg.data[0] << 8 | can_msg.data[1]
                        self.Tick[1] = can_msg.data[2] << 8 | can_msg.data[3]
                        self.Tick[2] = can_msg.data[4] << 8 | can_msg.data[5]
                        laser_int = can_msg.data[6] << 8 | can_msg.data[7]
                        #laser_float = map(laser_int,0,4096,0.5,10.0)   ##  from 0.5m to 10m
                                            ####################
                        pub_msg.data = [(self.Tick[0]), (self.Tick[1]), (self.Tick[2])]   
                        laser_msg.data = laser_int
                        self.laser_publisher_.publish(laser_msg)
                        self.publisher_.publish(pub_msg)
                else :
                    self.get_logger().error('time out on msg recv!!!')
                    finish_recv = False
            except can.CanOperationError :
                pass

def main(args=None):
    rclpy.init(args=args)

    can_class_test = can_class()

    rclpy.spin(can_class_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    can_class_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()