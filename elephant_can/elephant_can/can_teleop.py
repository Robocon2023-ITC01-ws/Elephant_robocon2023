import rclpy
from rclpy.node import Node
from kinematic_model.kinematic import *

# === ROS2 message ===
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

class ros_node(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        self.kinematic = kinematic()
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback,10)
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback,10)
        self.velocity_pub = self.create_publisher(Float32MultiArray, 'pub_speed', 10)
        self.velocity_timer = self.create_timer(0.05, self.velocity_callback)

    def joy_callback(self, joy_msg):
        self.vx = joy_msg.axes[1]
        self.vy = joy_msg.axes[0]
        #x = float(joy_msg.axes[4])
        #y = float(-joy_msg.axes[3])
        self.omega = joy_msg.axes[3]
      

    def twist_callback(self, twist_msg):
        self.vx = twist_msg.linear.x
        self.vy = twist_msg.linear.y
        self.omega = twist_msg.angular.z

    def velocity_callback(self):
        pub_msg = Float32MultiArray()
        Vx = self.kinematic.map(self.vx, -1, 1,-1,1)
        Vy = self.kinematic.map(self.vy, -1, 1, -1, 1)
        Vth = self.kinematic.map(self.omega, -1,1,-np.pi, np.pi)
        w1,w2,w3,w4 = self.kinematic.inverse_kinematic(Vx,Vy,Vth)
        pub_msg.data = [float (w1), float (w2), float (w3), float (w4)]
        data = np.array([w1, w2, w3, w4])
        print(data)
        self.velocity_pub.publish(pub_msg)
        


def main(args=None):
    rclpy.init(args=args)
    teleop_node = ros_node()
    rclpy.spin(teleop_node)
    teleop_node.destroy_node()
    rclpy.shutdown

if __name__=='__main__':
    main()
