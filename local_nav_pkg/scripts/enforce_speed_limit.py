#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit
from geometry_msgs.msg import Twist
import math

speed_limit = 1.0
linear_x = 0.0
linear_y = 0.0
linear_z = 0.0
angular_x = 0.0
angular_y = 0.0
angular_z = 0.0

class GoalPosePublisherNode(Node):

    def __init__(self):
        super().__init__('speed_limit_publisher')
        self.subscriber = self.create_subscription(SpeedLimit, 'speed_limit', self.speed_limit_callback, 10)
        self.subscriber2 = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel1', 10)
        self.goal_pose_timer_ = self.create_timer(2.0, self.publish_cmd_vel)
        
        
    def publish_cmd_vel(self):
        cmd_vel = Twist()
        if speed_limit < linear_x:
            cmd_vel.linear.x = speed_limit
        else:
            cmd_vel.linear.x = linear_x
        print(cmd_vel.linear.x)
        cmd_vel.linear.y = linear_y
        cmd_vel.linear.z = linear_z
        cmd_vel.angular.x = angular_x       
        cmd_vel.angular.y = angular_y
        cmd_vel.angular.z = angular_z
        self.publisher_.publish(cmd_vel)
        
    def speed_limit_callback(self, msg):
        global speed_limit
        speed_limit = msg.speed_limit

    def cmd_vel_callback(self, msg):
        global linear_x
        global linear_y
        global linear_z
        global angular_x
        global angular_y
        global angular_z
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        linear_z = msg.linear.z
        angular_x = msg.angular.x
        angular_y = msg.angular.y
        angular_z = msg.angular.z       

 
def main(args=None):  
    rclpy.init(args=args)
    node = GoalPosePublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
