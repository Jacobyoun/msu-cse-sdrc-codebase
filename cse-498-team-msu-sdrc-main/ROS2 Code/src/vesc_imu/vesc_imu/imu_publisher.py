import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUPublisher(Node):
    '''IMU publisher class from the Vesc'''
    def __init__(self):
        """Initialize the node"""
        super().__init__('imu_publisher')
        self.get_logger().info("IMU Publisher Node Started!")
        #subscribe to the topic to send data over
        self.create_subscription(
            Imu,
            '/sensors/imu/raw',
            self.publish_imu_data,
            10
        )
        
        self.publisher_ = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10
        )
    
    def publish_imu_data(self, msg: Imu):
        """Formatting the message and publish"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.x = msg.angular_velocity.x
        imu_msg.angular_velocity.y = msg.angular_velocity.y
        imu_msg.angular_velocity.z = msg.angular_velocity.z
        imu_msg.linear_acceleration.x = msg.linear_acceleration.x
        imu_msg.linear_acceleration.y = msg.linear_acceleration.y
        imu_msg.linear_acceleration.z = msg.linear_acceleration.z

        self.publisher_.publish(imu_msg)
        self.get_logger().info(f"IMU data published")

def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

