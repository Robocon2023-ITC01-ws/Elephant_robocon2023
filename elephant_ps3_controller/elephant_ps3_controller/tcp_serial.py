import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import urllib.request


############ function ###############
def map(Input, Min_Input, Max_Input, Min_Output, Max_Output):
    value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
    return value



class tcp_read(Node):
    def __init__(self):
        super().__init__('tcp_read')
        self.publisher_ = self.create_publisher(Joy, '/joy', 20)
        self.ps4_timer = self.create_timer(0.001, self.fc_callback)

        self.url = "http://192.168.0.100/" 



        self.joy_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def fc_callback(self):
        joy_msg = Joy()

        n = urllib.request.urlopen(self.url).read()
        n = n.decode("utf-8")
        data = [int(s) for s in n.split() if s.isdigit()]
        self.joy_data[0] = map(data[0],0,255,1.0,-1.0)
        self.joy_data[1] = map(data[1],0,255,1.0,-1.0)
        self.joy_data[2] = map(data[2],0,255,1.0,-1.0)
        self.joy_data[3] = map(data[3],0,255,1.0,-1.0)
        self.joy_data[4] = map(data[4],0,255,1.0,-1.0)
        self.joy_data[5] = map(data[5],0,255,1.0,-1.0)

        for i in range(6):
            if((self.joy_data[i] * self.joy_data[i]) < 0.0036):
                self.joy_data[i] = 0.0

        joy_msg.axes = [self.joy_data[0], self.joy_data[1], self.joy_data[2], self.joy_data[3],
                        self.joy_data[4], self.joy_data[5]]

        joy_msg.buttons = [data[6] >> 0 & 1, data[6] >> 1 & 1, data[6] >> 2 & 1,
                            data[6] >> 3 & 1, data[6] >> 4 & 1, data[6] >> 5 & 1, 0, 0,
                            data[6] >> 6 & 1, data[6] >> 7 & 1, data[7] >> 0 & 1,
                            data[7] >> 1 & 1, data[7] >> 2 & 1, data[7] >> 3 & 1,
                            data[7] >> 4 & 1, data[7] >> 5 & 1, data[7] >> 6 & 1,]
        
        self.publisher_.publish(joy_msg)

def main(args=None):
    rclpy.init(args=args)

    tcp_read_01 = tcp_read()

    rclpy.spin(tcp_read_01)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tcp_read_01.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try :
        main()
    except KeyboardInterrupt :
        pass