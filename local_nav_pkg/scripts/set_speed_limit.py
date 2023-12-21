#! /usr/bin/env python3


import rclpy
#import rospy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import math

goal_pose_x = 0.0
goal_pose_y = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0

class GoalPosePublisherNode(Node):

    def __init__(self):
        super().__init__('speed_limit_publisher')
        self.subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.subscriber2 = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        self.publisher_ = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.goal_pose_timer_ = self.create_timer(0.1, self.publish_speed_limit)
        
        
    def publish_speed_limit(self):
        x_dist = odometry_pose_x - goal_pose_x
        y_dist = odometry_pose_y - goal_pose_y
        dist = math.sqrt(x_dist*x_dist + y_dist*y_dist)
        
        speed_limit = SpeedLimit()
        speed_limit.header.frame_id = 'map'
        speed_limit.header.stamp.sec = 0
        speed_limit.percentage = False
        print(dist)
        if dist < 2:
            speed_limit.speed_limit = 0.1
        else:
            speed_limit.speed_limit = 5.0

        #speed_limit.speed_limit = 1.0
        self.publisher_.publish(speed_limit)

    def goal_pose_callback(self, msg):
        global goal_pose_x
        global goal_pose_y
        goal_pose_x = msg.pose.position.x
        goal_pose_y = msg.pose.position.y

    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
  	
 
def main(args=None):  
    rclpy.init(args=args)
    node = GoalPosePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
