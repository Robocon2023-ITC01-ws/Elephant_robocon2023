import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32



class pid_class(Node):
    def __init__(self):
        super().__init__('pid_test')
        self.timer_period = 0.02 # seconds
        self.prosses = self.create_timer(self.timer_period, self.prosses_callback)
        self.bno055_subscriber = self.create_subscription(Vector3, "/bno055/imu", self.bno055_callback, 20)
        self.omega_publisher = self.create_publisher(Float32,"/omega_control", 10)
        self.sub_input_yaw = self.create_subscription(Float32,"/input_yaw" ,self.reciece_yaw, 10)
        
        
        self.yaw = 0.0
        self.new_yaw = 0.0
        self.old_yaw = 0.0

        self.init_yaw = 1   #bool

        self.input_yaw = 0.0

    def bno055_callback(self, imu_msg):
        self.yaw = imu_msg.z
    
    def reciece_yaw(self, rx_yaw):
        self.input_yaw = rx_yaw.data

    def p_controller(self, kp, Input, Output):
        error = Input - Output
        power = kp * error
        
        return power
    
    def prosses_callback(self):
        if(self.init_yaw == 1):
            self.init_yaw = 0
            self.input_yaw = self.yaw
        else :
            omega_pub_msg = Float32()

            self.new_yaw = -1.0 *  self.p_controller(1.0, self.input_yaw, self.yaw)
            omega = (self.new_yaw - self.old_yaw )/self.timer_period
            print(self.new_yaw)
            omega_pub_msg.data = self.new_yaw
            self.omega_publisher.publish(omega_pub_msg)
            self.old_yaw = self.new_yaw

def main(args=None):
    rclpy.init(args=args)

    pid_node_test = pid_class()

    rclpy.spin(pid_node_test) 

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pid_node_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()