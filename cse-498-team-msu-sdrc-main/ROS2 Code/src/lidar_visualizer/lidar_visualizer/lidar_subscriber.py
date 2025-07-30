import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

import rosbag2_py

import time

class LidarSubscriber(Node):
    '''LiDAR subscriber class'''
    def __init__(self):
        super().__init__('lidar_subscriber')
        #subscribe to the topic
        self.subscription = self.create_subscription(LaserScan, 'scan', self.lidar_graph, 10)
        #create a plot to map out scan points
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.scat = self.ax.scatter([], [], s=5, c=[], cmap='Greys_r', lw=0)
        self.ax.set_rmax(5.0)
        

        #storing data to rosbag
        topic_info_bagger = rosbag2_py.TopicMetadata(
            name='lidar_bagging',
            type='sensor_msgs/msg/LaserScan', 
            serialization_format='cdr')          

    #graphing out data
    def lidar_graph(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        self.scat.set_offsets(np.c_[angles, ranges])
        self.scat.set_array(np.array(ranges))
        plt.draw()

        plt.pause(0.01)



def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

