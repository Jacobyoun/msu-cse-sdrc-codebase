import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from pynmeagps import NMEAReader
import io

class GNSSSubscriber(Node):
    '''GNSS subscriber class'''
    def __init__(self):
        """Initialize the Subscriber node"""
        super().__init__("gnss_subscriber")
        self.get_logger().info("GNSS Subscriber Node Started!")

        #subcribe to the topic
        self.subscription = self.create_subscription(String, "/gnss/raw_data", self.process_gnss_data, 10)
        self.gnss_publisher = self.create_publisher(NavSatFix, "/gnss/fix", 10)

    def process_gnss_data(self, msg):
        """Process the GNSS data"""
        try:
            nmea_bytes = msg.data.encode("utf-8")

            parsed = NMEAReader.parse(nmea_bytes)
            #parsing gnss data
            if hasattr(parsed, "lat") and hasattr(parsed, "lon"):
                nav_msg = NavSatFix()
                nav_msg.header.stamp = self.get_clock().now().to_msg()
                nav_msg.header.frame_id = "gps"
                nav_msg.latitude = parsed.lat
                nav_msg.longitude = parsed.lon
                nav_msg.altitude = getattr(parsed, "alt", 0.0)  # Default to 0 if no altitude data
                #publish message
                self.gnss_publisher.publish(nav_msg)
                self.get_logger().info(f"lat: {nav_msg.latitude}, lon: {nav_msg.longitude}")
            else:
                self.get_logger().info("No GPS data detected.")

        except Exception:
            self.get_logger().info("No valid GNSS data detected.")

def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = GNSSSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

