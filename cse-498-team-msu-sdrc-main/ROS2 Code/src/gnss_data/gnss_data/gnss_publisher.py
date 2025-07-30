import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from serial import Serial
from pynmeagps import NMEAReader

class GNSS_Publisher_Node(Node):
    ''' Publisher class of the GNSS'''
    def __init__(self):
        """Initialize the GNSS Node"""
        super().__init__('gps_reader_node')
        self.publisher_ = self.create_publisher(String, "/gnss", 1)
        # declare serial port, baudrate, and frequency of the gnss
        self.declare_parameter('serial_port', '/dev/gnss_device')  # COM5
        self.declare_parameter('baudrate', 4800)
        self.declare_parameter('timeout', 3.0)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        timeout = self.get_parameter('timeout').get_parameter_value().double_value

        #send message to terminal
        self.get_logger().info(f"Connecting to {serial_port} at {baudrate} baud")
        
        #accessing the GNSS
        try:
            self.stream = Serial(serial_port, baudrate, timeout=timeout)
            self.nmr = NMEAReader(self.stream)
            self.timer = self.create_timer(0.5, self.read_gps_data)
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.destroy_node()

    #Sending data to the subscriber
    def read_gps_data(self):
        """Read in the GNSS Data"""
        try:
            raw, parsed = self.nmr.read()
            if parsed and hasattr(parsed, 'lat') and hasattr(parsed, 'lon'):
                gnss_message = f"Latitude: {parsed.lat}, Longitude: {parsed.lon}"
                self.publisher_.publish(String(data=gnss_message))
                self.get_logger().info(gnss_message)
        except Exception as e:
            self.get_logger().error(f"Error reading GPS data: {e}")

def main(args=None):
    """Start the ROS node"""
    rclpy.init(args=args)
    node = GNSS_Publisher_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('GPS Reader Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
