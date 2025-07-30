import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
from sensor_msgs.msg import Imu

# This code is for if the user is on a system that supports the camera IMU

class IMUPublisher(Node):
    '''IMU publisher class'''
    def __init__(self):
        super().__init__('imu_publisher')
        self.get_logger().info("IMU Publisher Node Started!")

        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        #accessing the IMU on the camera
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        self.pipeline.start(self.config)

        self.timer = self.create_timer(0.1, self.publish_imu_data)

    def publish_imu_data(self):
        """Get the IMU data and publish it to the imu/data_raw topic"""
        #getting acceleration and gyro
        frames = self.pipeline.wait_for_frames()
        accel = frames[0].as_motion_frame().get_motion_data()
        gyro = frames[1].as_motion_frame().get_motion_data()

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        #format the message
        imu_msg.angular_velocity.x = gyro.x
        imu_msg.angular_velocity.y = gyro.y
        imu_msg.angular_velocity.z = gyro.z
        imu_msg.linear_acceleration.x = accel.x
        imu_msg.linear_acceleration.y = accel.y
        imu_msg.linear_acceleration.z = accel.z
        #sending data over
        self.imu_publisher.publish(imu_msg)
        self.get_logger().info(f"IMU data published")

def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

