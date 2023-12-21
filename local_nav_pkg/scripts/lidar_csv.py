#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
import point_cloud2
#import rospy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import csv

global cloud

class LidarDetectionNode(Node):

    def __init__(self):
        super().__init__('penguin_inference_publisher')
        self.subscriber = self.create_subscription(PointCloud2, 'velodyne_points', self.velodyne_callback, 10)
        
        self.publisher_ = self.create_publisher(PointCloud2, 'filtered_cloud', 1)
        self.penguin_inference_timer_ = self.create_timer(1, self.publish_cloud)

    def velodyne_callback(self, msg):
        global cloud
        #cloud = msg.data
        cloud = point_cloud2.read_points(msg)
                
    def publish_cloud(self):
        global cloud
        print('saving')
        f = open('test2.csv', 'w')
        writer = csv.writer(f)
        for point in cloud:
            #print(point)
            writer.writerow(point)        
        f.close()
        print('saved')


def main(args=None):
    rclpy.init(args=args)
    node = LidarDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()
