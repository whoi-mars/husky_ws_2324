#! /usr/bin/env python3

# Convert pointcloud obstacle readout to list of penguins

from xml.etree.ElementTree import tostring
import rclpy
import point_cloud2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from local_nav_pkg.msg import PenguinList
from zed_interfaces.msg import ObjectsStamped, Object
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import numpy as np
from sklearn.cluster import DBSCAN 

colors = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 212), (0, 128, 128), (220, 190, 255), (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195), (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128), (255, 255, 255), (0, 0, 0)]
class ObjectViz(Node):

    def __init__(self):
        super().__init__('zed_object_vizualizer')
        self.subscriber = self.create_subscription(ObjectsStamped, 'zed2/zed_node/obj_det/objects', self.objects_callback, 1)
        
        self.object_marker_publisher_ = self.create_publisher(MarkerArray, 'zed_obj_viz', 1)
        #self.cloud_timer_ = self.create_timer(.1, self.cluster)
        
    def objects_callback(self, msg):
        objects = msg.objects
        markers = MarkerArray()
        # id = 0
        for object in objects:
            marker = Marker()
            marker.id = int(object.label_id)
            # id += 1
            marker.ns = 'zed_obj'
            marker.type = 1
            marker.header.frame_id = 'base_link'
            lifetime = Duration()
            lifetime.nanosec = 200000000
            marker.lifetime = lifetime
            # print(object.position[0])
            marker.pose.position.x = float(object.position[0])
            marker.pose.position.y = float(object.position[1])
            marker.pose.position.z = float(object.position[2])
            # print(object.dimensions_3d)
            marker.scale.x = float(object.dimensions_3d[2])
            marker.scale.y = float(object.dimensions_3d[0])
            marker.scale.z = float(object.dimensions_3d[1])

            marker.color.b = 0.5
            marker.color.a = 1.0

            markers.markers.append(marker)
                       
        self.object_marker_publisher_.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectViz()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()