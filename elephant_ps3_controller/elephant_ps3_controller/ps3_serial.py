import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from pySerialTransfer import pySerialTransfer as txfer

class struct(object):
    Ljoy_x = 128
    Ljoy_y = 128
    Rjoy_x = 128
    Rjoy_y = 128
    Ljoy = 0
    Rjoy = 0
    BTN1 = 0
    BTN2 = 0

############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value



class serial_read(Node):
    def __init__(self):
        super().__init__('serial_read')
        self.testStruct = struct
        self.link = txfer.SerialTransfer('/dev/ttyUSB0',baud=1000000)
        self.link.open()
        self.publisher_ = self.create_publisher(Joy, '/joy', 20)
        self.ps4_timer = self.create_timer(0.001, self.fc_callback)

        self.joy_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def fc_callback(self):
        joy_msg = Joy()
        if self.link.available() :
            recSize = 0
            
            self.testStruct.Ljoy_x = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.Ljoy_y = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.Rjoy_x = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.Rjoy_y = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.Ljoy = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.Rjoy = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.BTN1 = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.testStruct.BTN2 = self.link.rx_obj(obj_type='B', start_pos=recSize)
            recSize += txfer.STRUCT_FORMAT_LENGTHS['c']

            self.joy_data[0] = map(self.testStruct.Ljoy_x,0,255,-1.0,1.0)
            self.joy_data[1] = map(self.testStruct.Ljoy_y,0,255,-1.0,1.0)
            self.joy_data[2] = map(self.testStruct.Rjoy_x,0,255,-1.0,1.0)
            self.joy_data[3] = map(self.testStruct.Rjoy_y,0,255,-1.0,1.0)
            self.joy_data[4] = map(self.testStruct.Ljoy,0,255,0.0,1.0)
            self.joy_data[5] = map(self.testStruct.Rjoy,0,255,0.0,1.0)

            for i in range(6):
                if((self.joy_data[i] * self.joy_data[i]) < 0.0036):
                    self.joy_data[i] = 0.0

            joy_msg.axes = [self.joy_data[0], self.joy_data[1], self.joy_data[2], self.joy_data[3],
                            self.joy_data[4], self.joy_data[5]]

            joy_msg.buttons = [self.testStruct.BTN1 >> 0 & 1, self.testStruct.BTN1 >> 1 & 1, self.testStruct.BTN1 >> 2 & 1,
                               self.testStruct.BTN1 >> 3 & 1, self.testStruct.BTN1 >> 4 & 1, self.testStruct.BTN1 >> 5 & 1,
                               self.testStruct.BTN1 >> 6 & 1, self.testStruct.BTN1 >> 7 & 1, self.testStruct.BTN2 >> 0 & 1,
                               self.testStruct.BTN2 >> 1 & 1, self.testStruct.BTN2 >> 2 & 1, self.testStruct.BTN2 >> 3 & 1,
                               self.testStruct.BTN2 >> 4 & 1, self.testStruct.BTN2 >> 5 & 1, self.testStruct.BTN2 >> 6 & 1,
                               self.testStruct.BTN2 >> 7 & 1]
            
            self.publisher_.publish(joy_msg)

        elif self.link.status < 0:
            if self.link.status == txfer.CRC_ERROR:
                print('ERROR: CRC_ERROR')
            elif self.link.status == txfer.PAYLOAD_ERROR:
                print('ERROR: PAYLOAD_ERROR')
            elif self.link.status == txfer.STOP_BYTE_ERROR:
                print('ERROR: STOP_BYTE_ERROR')
            else:
                print('ERROR: {}'.format(self.link.status))

def main(args=None):
    rclpy.init(args=args)

    serial_read_01 = serial_read()

    rclpy.spin(serial_read_01)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_read_01.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()