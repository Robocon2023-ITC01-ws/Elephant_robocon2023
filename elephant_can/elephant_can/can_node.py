import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import can 
############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value

class can_class(Node):
    def __init__(self):
        super().__init__('can_node_test')
        self.bus = can.interface.Bus(channel='can0', interface='socketcan',bitrate=1000000)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'feedback', 100)
        self.can_timer = self.create_timer(0.001, self.can_callback)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pub_speed',
            self.listener_callback,
            10)
        self.TxData = [128,0,128,0,128,0,128,0]
        
        self.V_back = [32768,32768,32768,32768]
        self.V_back_p = [0.0,0.0,0.0,0.0]
        self.yaw = 0
        self.pitch = 0
        self.ax = 0
        self.ay = 0
        ############
        
    def listener_callback(self, msg):
            V1_out  = (int)( map(msg.data[0], -100, 100, 0, 65536))
            V2_out  = (int)( map(msg.data[1], -100, 100, 0, 65536))
            V3_out  = (int)( map(msg.data[2], -100, 100, 0, 65536))
            V4_out  = (int)( map(msg.data[3], -100, 100, 0, 65536))
            self.TxData[0] = ((V1_out & 0xFF00) >> 8)
            self.TxData[1] = (V1_out & 0x00FF)
            self.TxData[2] = ((V2_out & 0xFF00) >> 8)
            self.TxData[3] = (V2_out & 0x00FF)
            self.TxData[4] = ((V3_out & 0xFF00) >> 8)
            self.TxData[5] = (V3_out & 0x00FF)
            self.TxData[6] = ((V4_out & 0xFF00) >> 8)
            self.TxData[7] = (V4_out & 0x00FF)
    def can_callback(self):
        pub_msg = Float32MultiArray()
        msg = can.Message(arbitration_id=0x111,
                data=self.TxData,
                is_extended_id=False)
        self.bus.send(msg,0.01)
        for i in range(2):
            can_msg = self.bus.recv(0.1)
            if(can_msg.arbitration_id != None):
                if can_msg.arbitration_id == 0x155:
                    self.V_back[0] = can_msg.data[0] << 8 | can_msg.data[1]
                    self.V_back[1] = can_msg.data[2] << 8 | can_msg.data[3]
                elif can_msg.arbitration_id == 0x140:
                    self.V_back[2] = can_msg.data[0] << 8 | can_msg.data[1]
                    self.V_back[3] = can_msg.data[2] << 8 | can_msg.data[3]
                                        ####################
                    self.V_back_p[0] = map(self.V_back[0],0,65536,-100,100)
                    self.V_back_p[1] = map(self.V_back[1],0,65536,-100,100)
                    self.V_back_p[2] = map(self.V_back[2],0,65536,-100,100)
                    self.V_back_p[3] = map(self.V_back[3],0,65536,-100,100)
                # elif can_msg.arbitration_id == 0x010:
                #     back_yaw = can_msg.data[0] << 8 | can_msg.data[1]
                #     back_pitch = can_msg.data[2] << 8 | can_msg.data[3]
                #     back_ax = can_msg.data[4] << 8 | can_msg.data[5]
                #     back_ay = can_msg.data[6] << 8 | can_msg.data[7]

                #     self.yaw = map(back_yaw,0,65536,0,360)
                #     self.pitch = map(back_pitch,0,65536,0,360)
                #     self.ax = map(back_ax,0,65536,-100,100)
                #     self.ay = map(back_ay,0,65536,-100,100)

        for i in range(4):
            if self.V_back_p[i] < 0.01 and self.V_back_p[i] > -0.01:
                self.V_back_p[i] = 0
        pub_msg.data = [(float)(self.V_back_p[0]),(float) (self.V_back_p[1]),(float) (self.V_back_p[2]),(float)(self.V_back_p[3])]
                        
        self.publisher_.publish(pub_msg)



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
