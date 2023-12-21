#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math

class PenguinInferenceNode(Node):

    def __init__(self):
        super().__init__('penguin_inference_publisher')
        self.publisher_ = self.create_publisher(String, 'inference', 1)
        self.goal_pose_timer_ = self.create_timer(1, self.publish_inference)

    def publish_inference(self):
        with open("/home/administrator/TensorFlow/penguin_detection_model/inference.py") as f:
            exec(f.read())
        self.publisher_.publish('toot')
        
def main(args=None):  
    rclpy.init(args=args)
    node = PenguinInferenceNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
