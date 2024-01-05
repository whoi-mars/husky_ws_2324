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

class ObjectViz(Node):

    def __init__(self):
        super().__init__('zed_object_vizualizer')
        self.subscriber = self.create_subscription(ObjectsStamped, 'zed2/zed_node/obj_det/objects', self.objects_callback, 1)
        
        self.object_marker_publisher_ = self.create_publisher(MarkerArray, 'zed_obj_viz_array2', 1)
        #self.cloud_timer_ = self.create_timer(.1, self.cluster)
        
    def objects_callback(self, msg):
        objects = msg.objects
        markers = MarkerArray()
        # id = 0
        print('objects: ', len(objects))
        print_list = []
        for object in objects:
                
            if not np.isnan(object.position[0]):
                if float(object.dimensions_3d[1]) > 0.25:
                    print(float(object.dimensions_3d[1])) 
            
                    print_list.append([object.sublabel, object.confidence])
                    # if object.sublabel == 'Bird':
                    # print(object.position[0])
                    marker = Marker()
                    marker.id = int(object.label_id)
                    # id += 1
                    marker.ns = 'zed_obj'
                    marker.type = 1
                    marker.header.frame_id = 'zed2_camera_center'
                    lifetime = Duration()
                    lifetime.nanosec = 300000000
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
                    if object.sublabel == 'Bird':
                        marker.color.g = 0.75
                    if object.sublabel == 'Person':
                        marker.color.r = 0.75
                    marker.color.a = 1.0

                    markers.markers.append(marker)
        print(print_list)
        # print('publishing')               
        self.object_marker_publisher_.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectViz()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()