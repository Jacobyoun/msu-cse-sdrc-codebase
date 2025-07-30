import pyrealsense2 as rs
import numpy as np
import cv2

#look at ROS2 Code directory for full commenting

def generate_heatmap(depth_image):
    # depth_float = depth_image.astype(np.float32)
    # scaled = (depth_float / 5000.0) * 255.0
    # scaled = np.clip(scaled, 0, 255)
    # normalized = scaled.astype(np.uint8)
    # return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


    depth_float = np.where(depth_image == 0, np.nan, depth_image.astype(np.float32))

    span = 10000.0 #span is the full range of colors in the heatmap. 0 is most blue (closest), 5000 is most red (farthest)
    scaled = (depth_float / span) * 255.0

    scaled = np.clip(scaled, 0, 255)
    normalized = np.nan_to_num(scaled, nan=0).astype(np.uint8)

    # normalized = 255 - normalized #flip the colors if you want
    return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    pipeline.start(config)
    print("Depth Heatmap Started")

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            if not depth_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data())
            print("Received a depth image")

            heatmap = generate_heatmap(depth_image)

            cv2.imshow("Depth Heatmap", heatmap)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Depth Heatmap Stopped")

if __name__ == "__main__":
    main()




















# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge


# class DepthHeatmapOnlyNode(Node):
#     def __init__(self):
#         super().__init__("depth_heatmap_only_node")

#         # Subscribe to depth camera topic
#         self.depth_sub = self.create_subscription(
#             Image, "/camera/depth/image_raw", self.depth_callback, 10
#         )

#         # Publisher for heatmap
#         self.heatmap_pub = self.create_publisher(
#             CompressedImage, "/camera/depth/heatmap/compressed", 10
#         )

#         self.bridge = CvBridge()
#         self.get_logger().info("🟢 Depth Heatmap Node Started")

#     def depth_callback(self, msg):
#         self.get_logger().info("📸 Received a depth image!")

#         try:
#             depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")

#             if depth_image is None:
#                 self.get_logger().error("⚠️ Depth image conversion failed!")
#                 return

#             # Generate heatmap
#             heatmap = self.generate_heatmap(depth_image)

#             # Convert to compressed image message
#             heatmap_msg = self.bridge.cv2_to_compressed_imgmsg(heatmap, dst_format="jpeg")
#             self.heatmap_pub.publish(heatmap_msg)
#             self.get_logger().info("🚀 Published heatmap to /camera/depth/heatmap/compressed")

#         except Exception as e:
#             self.get_logger().error(f"❌ Error processing depth image: {e}")

#     def generate_heatmap(self, depth_image):
#         normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
#         normalized = np.uint8(normalized)
#         return cv2.applyColorMap(normalized, cv2.COLORMAP_JET)


# def main(args=None):
#     rclpy.init(args=args)
#     node = DepthHeatmapOnlyNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()