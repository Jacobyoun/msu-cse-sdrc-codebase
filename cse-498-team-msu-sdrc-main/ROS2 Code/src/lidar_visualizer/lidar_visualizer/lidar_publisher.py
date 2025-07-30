import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar
import numpy as np
import threading
import time

#declare port, baudrate, and frequency of LiDAR
PORT = "/dev/ttyUSB0"  
BAUDRATE = 115200
TIMEOUT = 1

#due to the lidar now spinning faster than before we had to redo some of the code to adapt to the speed

class LidarPublisher(Node):
    '''Lidar publisher class'''
    def __init__(self):
        """Initialize the node"""
        super().__init__('lidar_publisher')
        self.publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        #Connecting to the LiDAR
        self.lidar = RPLidar(PORT, baudrate=BAUDRATE, timeout=TIMEOUT)
        self.lidar.start_motor()
        time.sleep(1)  

        #as you can see instead of adding a timer to publish we are now using a thread.
        #adding a timer before clogged the system and was unreliable. it would publish and restart the scan pooints
        #every 10 ms even if the lidar didn't complete full rotation (360 degrees)
        #the threead allows it to fully rotate 360 degrees. it is now event based in a way since it will call the for loop
        #again after completing a full rotation
        thread = threading.Thread(target=self.publish_scan_loop, daemon=True)
        thread.start()

    def publish_scan_loop(self):
        """Publish the scan"""
        try:
            #iter_scans returns a generator every full rotation containing all of the distances,angles of all scan points
            #it will complete a full rotation because of the thread running here infinitely until we ctrl c
            for scan_data in self.lidar.iter_scans(max_buf_meas=500):
                #gather scan points and add them to a list
                msg = LaserScan()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "lidar_frame"

                angles = [np.radians(p[1]) for p in scan_data]
                distances = [p[2] / 1000.0 for p in scan_data]

                if angles:
                    msg.angle_min = min(angles)
                    msg.angle_max = max(angles)
                else:
                    msg.angle_min = 0.0
                    msg.angle_max = 0.0
                #min range so that the LiDAR doesn't produce jibberish LiDAR detecting itself
                msg.range_min = 0.16  
                msg.range_max = 4.0  
                #send the distances as a message
                msg.ranges = distances
                #sendes the angles as a message
                msg.intensities = angles  
                #publishing data
                self.publisher_.publish(msg)

                self.get_logger().info(f"Published Lidar Data: {len(msg.ranges)} points")
        except Exception as e:
            self.get_logger().error(f"Error while sending Lidar data: {e}")

#thread will end here when we ctrl c
    def destroy_node(self):
        """Cleanly close the LiDAR"""
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        super().destroy_node()

def main(args=None):
    """Start the node"""
    rclpy.init(args=args)
    node = LidarPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
 