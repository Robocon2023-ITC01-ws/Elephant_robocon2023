import sys

from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node
import time


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('shooter_client')
        self.cli = self.create_client(SetBool, 'shooter_srv')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        print(self.req.data)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    if(int(sys.argv[1]) == 1):
        data = True
        response = minimal_client.send_request(data)
        time.sleep(3)
        data = False
        response = minimal_client.send_request(data)
    else:
        data = False
        response = minimal_client.send_request(data)
    minimal_client.get_logger().info(
        'Result : for %d %s' %
        (bool(sys.argv[1]), response.message))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()