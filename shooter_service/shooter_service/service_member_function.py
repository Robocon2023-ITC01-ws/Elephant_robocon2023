import rclpy
from rclpy.node import Node
from shooter_interfaces.srv import Shooterdata
from std_msgs.msg import Float32
from time import sleep
try :
    from shooter_service import shooter_lib
except :
    pass
try :
    import shooter_lib
except :
    pass

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Shooterdata, 'shooting_data', self.shooting_callback)
        self.pub_speed = self.create_publisher(Float32,'shooter',10)

    def shooting_callback(self, request, response):
        pub_msg = Float32()
        if request.distance < 1.0 :
            speed = shooter_lib.shooter(type="EE", type_pole="type_2", distance = request.distance).shooter1()
        elif request.distance > 1.0 and request.distance < 3.2 :
            speed = shooter_lib.shooter(type="EE", type_pole="type_3", distance = request.distance).shooter1()
        elif request.distance > 3.2 :
            speed = shooter_lib.shooter(type="EE", type_pole="type_1", distance = request.distance).shooter1()
        else :
            speed = 0
        
        pub_msg.data = -1.0 * (float)(speed)
        self.pub_speed.publish(pub_msg)
        sleep(2)
        pub_msg.data = 1.0 * (float)(speed)
        self.pub_speed.publish(pub_msg)
        sleep(1)
        pub_msg.data = 0.0
        self.pub_speed.publish(pub_msg)


        self.get_logger().info('Incoming request  %d' % (request.distance))
        response.goal = True
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()