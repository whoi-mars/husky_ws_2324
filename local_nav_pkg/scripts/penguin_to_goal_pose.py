#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from local_nav_pkg.msg import Penguin
from local_nav_pkg.msg import PenguinList
import math

list_of_points = []
penguin_list = []
penguin_label_count = 0

x_goal = 0.0
y_goal = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0

class GoalPosePublisherNode(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.subscriber = self.create_subscription(PenguinList, 'penguin_list',self.penguin_list_callback,10)
        self.subscriber2 = self.create_subscription(Odometry, 'odometry/filtered', self.odometry_callback, 10)
        
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose_timer_ = self.create_timer(0.1, self.publish_goal_pose)
        
    
        
    def penguin_list_callback(self, msg):
        global x_goal
        global y_goal
        if len(msg.penguins) > 0:
            x_goal = msg.penguins[0].point.x
            y_goal = msg.penguins[0].point.y

    def publish_goal_pose(self):     
        global x_goal
        global y_goal
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'velodyne'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = x_goal
        goal_pose.pose.position.y = y_goal
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        #speed_limit.speed_limit = 1.0
        self.publisher_.publish(goal_pose)
        

    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
  	
 
def main(args=None):  
    rclpy.init(args=args)
    print('Start')
    node = GoalPosePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
