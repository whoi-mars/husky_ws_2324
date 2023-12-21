#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPosePublisherNode(Node):

    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.goal_pose_timer_ = self.create_timer(
            2.0, self.publish_goal_pose)
        

    def publish_goal_pose(self):
    
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'odom'
        goal_pose.header.stamp.sec = 0
        goal_pose.pose.position.x = 5.0
        goal_pose.pose.position.y = 3.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        
        #goal_pose_data = '{header: {stamp: {sec: 0}, frame_id: \'map\'}, pose: {position: {x: 10.0, y: 10.0, z: 0.0}, orientation: {w: 1.0}}}'
        #msg = PoseStamped()
        #msg.data = goal_pose_data
        self.publisher_.publish(goal_pose)
 
def main(args=None):
    rclpy.init(args=args)
    node = GoalPosePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
