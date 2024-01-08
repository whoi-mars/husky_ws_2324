#! /usr/bin/env python3

# Visualize the prnguin list

import rclpy
from rclpy.node import Node
from local_nav_pkg.msg import PenguinList
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration

colors = [(230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200), (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230), (210, 245, 60), (250, 190, 212), (0, 128, 128), (220, 190, 255), (170, 110, 40), (255, 250, 200), (128, 0, 0), (170, 255, 195), (128, 128, 0), (255, 215, 180), (0, 0, 128), (128, 128, 128), (255, 255, 255), (0, 0, 0)]
class PenguinViz(Node):

    def __init__(self):
        super().__init__('penguin_visualizer')
        self.subscriber = self.create_subscription(PenguinList, 'penguin_list', self.list_callback, 1)
        
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'penguin_viz', 1)

    def list_callback(self, msg):
        penguins = msg.penguins
        markers = MarkerArray()
        # id = 0
        for penguin in penguins:
            marker = Marker()
            marker.id = int(penguin.label)
            # id += 1
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
            # give each unvisited penguin its own color for tracking
            if penguin.visited == 'unvisited':
                color = colors[marker.id%20]
                marker.color.r = float(color[0])/255
                marker.color.b = float(color[1])/255
                marker.color.g = float(color[2])/255
                marker.color.a = 0.5 - 0.5*penguin.age/40
            elif penguin.visited == 'completed':
                marker.color.g = 1.0
                marker.color.a = 1.0
            else:
                marker.color.r = 1.0
                marker.color.a = 1.0

            markers.markers.append(marker)
                       
        self.marker_publisher_.publish(markers)

def main(args=None):
    rclpy.init(args=args)
    node = PenguinViz()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__ == "__main__":
	main()