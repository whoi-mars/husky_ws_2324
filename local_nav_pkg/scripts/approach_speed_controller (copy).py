#! /usr/bin/env python3

import rclpy
#import rospy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from local_nav_pkg.msg import PenguinApproachStatus
import math
import time

goal_pose_x = 0.0
goal_pose_y = 0.0
odometry_pose_x = 0.0
odometry_pose_y = 0.0
odometry_pose_theta = 0.0
goal_reached = False
arrival_time = 0
current_penguin_label = str(0)
approach_status = "pending"

class HuskyVelocityPublisherNode(Node):

    def __init__(self):
        super().__init__('husky_velocity_publisher')
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, 'goal_pose', self.goal_pose_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)

        self.velocity_publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)

        self.velocity_timer_ = self.create_timer(10, self.publish_husky_velocity)
        self.approach_status_timer_ = self.create_timer(10, self.publish_approach_status)
        
    def goal_pose_callback(self, msg):
        print('new data')
        global goal_pose_x
        global goal_pose_y
        goal_pose_x = msg.pose.position.x
        goal_pose_y = msg.pose.position.y
        
    def odometry_callback(self, msg):
        global odometry_pose_x
        global odometry_pose_y
        global odometry_pose_theta
        odometry_pose_x = msg.pose.pose.position.x
        odometry_pose_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        
        #(roll, pitch, odometry_pose_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
        siny_cosp = 2 * (rot_q.w * rot_q.z + rot_q.x * rot_q.y)
        cosy_cosp = 1 - 2 * (rot_q.y * rot_q.y + rot_q.z * rot_q.z)
        odometry_pose_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def approach_status_callback(self, msg):
        global current_penguin_label
        global approach_status
        current_penguin_label = msg.label
        approach_status = msg.status

    def publish_husky_velocity(self):
        #x_dist = odometry_pose_x - goal_pose_x
        #y_dist = odometry_pose_y - goal_pose_y
        x_dist = -goal_pose_x
        y_dist = -goal_pose_y
        
        dist = math.sqrt(x_dist*x_dist + y_dist*y_dist)

        global goal_reached
        global approach_status
        global arrival_time
        global current_penguin_label
        speed = Twist()
       
        angle_to_goal = math.atan2(-y_dist, -x_dist)
        print('Angle to goal: ', round(angle_to_goal,2),'Pose theta: ', round(odometry_pose_theta,2))
        print('Distance to goal in x: ', round(x_dist,2),'Distance to goal in y: ', round(y_dist,2))
                
        if current_penguin_label == str(0):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if approach_status == "pending":
                approach_status = "engaging"
            if approach_status == "engaging":	
                #if angle_to_goal - odometry_pose_theta > 0.1 and angle_to_goal - odometry_pose_theta < math.pi:
                if angle_to_goal > 0.1 and angle_to_goal < math.pi:
                    print('Adjusting Angle - positive')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.5
                #elif angle_to_goal - odometry_pose_theta < -0.1 and angle_to_goal - odometry_pose_theta > -math.pi:
                elif angle_to_goal  < -0.1 and angle_to_goal > -math.pi:
                    print('Adjusting Angle - negative')
                    speed.linear.x = 0.0
                    speed.angular.z = -0.5
                elif dist > 4.0:
                    print('Fast Approach')
                    speed.linear.x = 1.0
                    speed.angular.z = 0.0
                elif dist > 2.5:
                    print('Moderate Approach')
                    speed.linear.x = 0.5
                    speed.angular.z = 0.0
                elif dist > 1.5:
                    print('Slow Approach')
                    speed.linear.x = 0.25
                    speed.angular.z = 0.0
                else:
                    print('Goal Reached!')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    goal_reached = True
                    arrival_time = self.get_clock().now().nanoseconds
                    approach_status = "scanning"
            if approach_status == "scanning":
                wait_time = 10 * 10**9
                if self.get_clock().now().nanoseconds < arrival_time + wait_time:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    approach_status = "scanning"
                else:
                    approach_status = "disengaging"
            if approach_status == "disengaging":
                if dist > 3.0:
                    print('Backed Away')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    goal_reached = False
                    approach_status = "complete"
                elif dist > 2.0:
                    print('Moderate Disengagement')
                    speed.linear.x = -1.0
                    speed.angular.z = 0.0
                elif dist > 0.0:
                    print('Slow Disengage')
                    speed.linear.x = -0.5
                    speed.angular.z = 0.0
                    
        #speed.linear.x = 0.5
        #speed.linear.y = 0.0
        #speed.linear.z = 0.0
        
        #speed.angular.x = 0.0
        #speed.angular.y = 0.0
        #speed.angular.z = 0.0
        
        
        #speed_limit.speed_limit = 1.0
        self.velocity_publisher_.publish(speed)

    def publish_approach_status(self):
        global current_penguin_label
        global approach_status

        penguin_status = PenguinApproachStatus()
        penguin_status.label = current_penguin_label
        penguin_status.status = approach_status
        self.approach_status_publisher_.publish(penguin_status)
    	
 
def main(args=None):
    rclpy.init(args=args)
    node = HuskyVelocityPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
