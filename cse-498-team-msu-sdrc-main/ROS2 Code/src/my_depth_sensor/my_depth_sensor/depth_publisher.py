#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

# This ROS node is not currently being used because the software it requires is not compatible with Jetpack 6.x
# We created this implementation so that if they use a different hardware system they can use it. Or if Nvidia patches the issue later.
# Realsence D435i camera

class DepthHeatmapOnlyNode(Node):
    """ROS node class for the heat map"""
    def __init__(self):
        """Initalize the node"""
        super().__init__("depth_heatmap_only_node")

        #this make the dynamic input of the heatmap possible (changing distance live)
        self.declare_parameter("span", 5000.0)
        
        #it's subscribed to a publisher from the depth camera (infrared) that helps it recieve the distances per pixel
        self.depth_sub = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.depth_callback, 10)
        self.heatmap_pub = self.create_publisher(CompressedImage, "/camera/depth/heatmap/compressed", 10)

        self.bridge = CvBridge()
        self.get_logger().info("Depth Heatmap Node Started")

    def depth_callback(self, msg):
        """Receive teh image"""
        self.get_logger().info("Received a depth image")

        try:
            #this is very important. it brings the distances per pixel and adds them to a dynamic array
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

            if depth_image is None:
                self.get_logger().error("Depth image conversion failed")
                return

            heatmap = self.generate_heatmap(depth_image)
            if heatmap is None:
                self.get_logger().error("Heatmap generation failed")
                return

            heatmap_msg = self.bridge.cv2_to_compressed_imgmsg(heatmap, dst_format="jpeg")
            self.heatmap_pub.publish(heatmap_msg)
            self.get_logger().info("Published heatmap to /camera/depth/heatmap/compressed")

        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def generate_heatmap(self, depth_image):
        """this is the heart of the sensor publisher. it takes in dynamic input distances. it takes in"""
        
        span = self.get_parameter("span").get_parameter_value().double_value

        
        depth_float = np.where(depth_image == 0, np.nan, depth_image.astype(np.float32))
        scaled = (depth_float / span) * 255.0
        scaled = np.clip(scaled, 0, 255)

        #this entire calculatiom is linear in terms of the distance and color   x=distance, y=color
        #y=x
        normalized = np.nan_to_num(scaled, nan=0).astype(np.uint8)

        normalized = 255 - normalized #flip the colors if you want

        #adds the actual color depending on the distance of the pixel
        return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = DepthHeatmapOnlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()