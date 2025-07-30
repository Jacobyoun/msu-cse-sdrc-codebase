#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage 
from cv_bridge import CvBridge

class CameraFeedPublisher(Node):
    """Class for the ROS2 camera node"""
    def __init__(self):
        """Initialize the node"""
        super().__init__('camera_publisher')

        self.publisher = self.create_publisher(CompressedImage, '/camera_feed', 10)

        self.timer = self.create_timer(1/30, self.publish_frame)

        #this captures the index/USB port on the car. it's set to automatically detecy
        #the camera from any port
        self.cap = cv2.VideoCapture('/dev/video4', cv2.CAP_V4L2)

        #some terminal messages for debugging
        if not self.cap.isOpened():
            self.get_logger().error("failed to open camera check if path is correct and camera is connected.")
        else:
            self.get_logger().info("camera opened successfully.")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 10)
        
        self.bridge = CvBridge()
        
        self.get_logger().info("camera publisher Started")

    def publish_frame(self):
        """Publish frame"""
        #reads in the frames from the camera
        ret, frame = self.cap.read()
        #im resizing the camera to 320 b 240 to reduce the lagginess of the camear
        frame = cv2.resize(frame, (320, 240))
        if not ret:
            self.get_logger().warn("failed to read frame from camera.")
            return


        if ret:
            #im compressing the jpeg frames from the camera so that it runs faster
            compressed_msg = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpeg")
            
            #sending it to the laptop subscriber
            self.publisher.publish(compressed_msg)
            self.get_logger().info("Publishing Camera")

    def destroy_node(self):
        """Cleanly close the camera"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = CameraFeedPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera feed node")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
