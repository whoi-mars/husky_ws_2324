#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from local_nav_pkg.msg import PenguinList
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
from sklearn.cluster import DBSCAN 

class PenguinViz(Node):

    def __init__(self):
        super().__init__('penguin_visualizer')
        self.subscriber = self.create_subscription(PenguinList, 'penguin_list', self.list_callback, 1)
        
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'penguin_viz', 1)
        #self.cloud_timer_ = self.create_timer(.1, self.cluster)
        
    def list_callback(self, msg):
        penguins = msg.penguins
        markers = MarkerArray()
        id = 0
        for penguin in penguins:
            marker = Marker()
            marker.id = id
            id += 1
            marker.ns = 'penguin'
            marker.type = 1
            marker.header.frame_id = 'velodyne'
            lifetime = Duration()
            lifetime.nanosec = 200000000
            marker.lifetime = lifetime
            marker.pose.position.x = penguin.point.x
            marker.pose.position.y = penguin.point.y
            marker.pose.position.z = penguin.point.z

            marker.scale.x = penguin.scale.x
            marker.scale.y = penguin.scale.y
            marker.scale.z = penguin.scale.z

            marker.color.b = 1.0
            marker.color.a = .5 - .5*penguin.age/40
            markers.markers.append(marker)
                       
        self.marker_publisher_.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PenguinViz()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()