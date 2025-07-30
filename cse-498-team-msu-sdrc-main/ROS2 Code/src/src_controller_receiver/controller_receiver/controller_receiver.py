import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import time

# Set the max values for the brake and speed
BRAKE_MAX = 200000.0
SPEED_MAX = 6000.0

# Global variable to set the amount of time the car will wait for a command before auto-braking
MAX_TIMEOUT = 0.5

class ControllerReceiver(Node):
    """
    ROS2 Node class that subscribes to data from the /controller_data topic and publishes
    to /commands/motor/speed, /commands/servo/position, /commands/motor/brake
    ---
    Attributes:
    subscription : self.create_subscription(Vector3, '/controller_data', self.listener_callback, 10)
    timer : self.create_timer(0.15, self.check_timeout)
    speed_publisher : self.create_publisher(Float64, '/commands/motor/speed', 10)
    steering_publisher : self.create_publisher(Float64, '/commands/servo/position', 10)
    brake_publisher : self.create_publisher(Float64, '/commands/motor/brake', 10)
    ---
    Member Functions:
    __init__ (self) : Initialize the node
    listener_callback : Callback function that receives controller input and sends VESC commands to vesc_driver
    check_timeout
    """
    def __init__(self):
        """Initialize the node"""
        super().__init__('controller_receiver')

        self.declare_parameter("speed_max", SPEED_MAX)

        # Get speed_max parameter with a default value if not set
        self.speed_max = self.get_parameter("speed_max").value
        self.get_logger().info(f"Using Speed Max: {self.speed_max}")

        # Subscribe to the controller_data topic
        self.subscription = self.create_subscription(
            Vector3,
            '/controller_data',
            self.listener_callback,
            10
        )

        # Publisher for motor speed
        self.speed_publisher = self.create_publisher(Float64, '/commands/motor/speed', 10)

        # Publisher for steering angle
        self.steering_publisher = self.create_publisher(Float64, '/commands/servo/position', 10)

        # Publisher for braking
        self.brake_publisher = self.create_publisher(Float64, '/commands/motor/brake', 10)

        self.get_logger().info("Controller Receiver Node Started")

        # Set the max timeout time
        # If the controller_subscriber does not see a message posted to any of the above topics it will automatically brake
        self.last_received_time = time.time()
        self.timeout_seconds = MAX_TIMEOUT  # Set timeout duration

        # Create a timer to check for timeout every .15 seconds
        self.timer = self.create_timer(0.15, self.check_timeout)

    def listener_callback(self, msg):
        """Callback function that receives controller input and sends VESC commands to the vesc_driver node"""

        ###
        # msg.x = speed; msg.y = brake; msg.z = steering angle
        ###

        # Update the last time that input data was received
        self.last_received_time = time.time()

        # Convert gas input to speed (adjust scale as needed)
        speed_msg = Float64()
        brake_msg = Float64()
        steering_msg = Float64()

        # Use if statement to set a dead zone for the gas is barely being pressed
        if (msg.x > 2):
            speed_msg.data = msg.x * (self.speed_max / 100)  # Adjust multiplier with parameter value
            self.speed_publisher.publish(speed_msg)
            self.get_logger().info(f"Published Speed: {speed_msg.data}")

        # Convert steering input to VESC-compatible range (0 to 1)
        steering_msg.data = ((msg.z + 30) / 60)
        self.steering_publisher.publish(steering_msg)
        self.get_logger().info(f"Published Steering: {steering_msg.data}")

        # Use if statement to set a dead zone for the brake is barely being pressed
        if (msg.y > 2):
            brake_msg.data = (msg.y / 100 * BRAKE_MAX)
            self.brake_publisher.publish(brake_msg)
            self.get_logger().info(f"Published Brake: {speed_msg.data}")

    def check_timeout(self):
        """Checks if it has been over self.timeout_seconds time since the last command was received"""
        # Make sure current time - last received time is greater than the timeout seconds rate
        if time.time() - self.last_received_time > self.timeout_seconds:
            brake_msg = Float64()
            brake_msg.data = 100000.0
            self.brake_publisher.publish(brake_msg)
            self.get_logger().warn("Connection lost: Applying brake")


def main(args=None):  #
    """Main function to start the node"""
    rclpy.init(args=args)
    node = ControllerReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
