import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# #testing only
# import time


class LoopbackTestCar(Node):
    """Loopback node class"""
    def __init__(self):
        """Initialize the node"""
        super().__init__('loopbacktest_laptop')

        self.publisher = self.create_publisher(String, 'loopback_pong', 10)
        self.subscriber = self.create_subscription(String, 'loopback_ping', self.get_pong, 10)

    def get_pong(self, msg):
        """Receive message from the other node and publish it so that the website receives the message"""
        # time.sleep(1)
        self.publisher.publish(msg)    

def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    loopbackCar = LoopbackTestCar()
    rclpy.spin(loopbackCar)

    loopbackCar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()