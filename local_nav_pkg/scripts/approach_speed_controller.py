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

import datetime
import logging
import sys
import time
from logging.handlers import TimedRotatingFileHandler
from rfid_reader.utils import get_lock
from getmac import get_mac_address
from rfid_reader.utils_usb import usb_reader_open, usb_reader_close, usb_cmd


RFID_READER_USB_PORT = '/dev/ttyUSB0'


mac = get_mac_address().replace(':', "-")
now = datetime.datetime.now().strftime('%Y%m%d')
path = 'husky_ws2/src/local_nav_pkg/scripts/rfid_reader/logs/{}_rfid_daemon_{}.log'.format(now, mac)
lg = logging.getLogger(path)
lg.setLevel(logging.DEBUG)
han = TimedRotatingFileHandler(filename=path, when='midnight', interval=1)
fmt = logging.Formatter("%(asctime)s | %(message)s", '%Y%m%d-%H%M%S')
han.setFormatter(fmt)
lg.addHandler(han)

SLOWEST_SPEED = 0.01
MEDIUM_SPEED = 0.025
HIGH_SPEED = 0.1

TURN_SPEED = 0.025

# test speeds
# SLOWEST_SPEED = 0.04
# MEDIUM_SPEED = 0.2
# HIGH_SPEED = .4

# TURN_SPEED = .2

class HuskyVelocityPublisherNode(Node):

    def __init__(self):
        super().__init__('husky_velocity_publisher')
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, 'local_goal_pose', self.goal_pose_callback, 10)
        self.odometry_subscriber = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)

        self.velocity_publisher_ = self.create_publisher(Twist, 'approach_vel', 10)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)

        self.velocity_timer_ = self.create_timer(.05, self.publish_husky_velocity)
        self.approach_status_timer_ = self.create_timer(.1, self.publish_approach_status)

        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0

        self.odometry_pose_x = 0.0
        self.odometry_pose_y = 0.0
        self.odometry_pose_theta = 0.0
        self.goal_reached = False
        self.arrival_time = 0
        self.current_penguin_label = str(0)
        self.approach_status = "pending"
        
        
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        
        
    def odometry_callback(self, msg):
        self.odometry_pose_x = msg.pose.pose.position.x
        self.odometry_pose_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        
        #(roll, pitch, odometry_pose_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        
        siny_cosp = 2 * (rot_q.w * rot_q.z + rot_q.x * rot_q.y)
        cosy_cosp = 1 - 2 * (rot_q.y * rot_q.y + rot_q.z * rot_q.z)
        self.odometry_pose_theta = math.atan2(siny_cosp, cosy_cosp)
    
    def approach_status_callback(self, msg):
        self.current_penguin_label = msg.label
        self.approach_status = msg.status

    def publish_husky_velocity(self):
        x_dist1 = -self.goal_pose_x
        y_dist1 = -self.goal_pose_y 
        x_dist = x_dist1
        y_dist = y_dist1

        dist = -x_dist

        speed = Twist()
       
        angle_to_goal = math.atan2(-y_dist1, -x_dist1)
        print('Angle to goal: ', round(angle_to_goal,2),'Pose theta: ', round(self.odometry_pose_theta,2))
        #print('Distance to goal in x: ', round(x_dist,2),'Distance to goal in y: ', round(y_dist,2))
        print('Distance to goal in x: ', round(dist,2), self.approach_status)        
        if self.current_penguin_label == str(0):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            if self.approach_status == "pending":
                self.approach_status = "engaging"
            if self.approach_status == "engaging":	
                #if angle_to_goal - odometry_pose_theta > 0.1 and angle_to_goal - odometry_pose_theta < math.pi:
                if dist < 1.0:
                    if abs(angle_to_goal) > 0.75:
                        print('Too close with bad angle - backing up')
                        speed.linear.x = -SLOWEST_SPEED
                        speed.angular.z = 0.0
                    else:
                        print('Goal Reached!')
                        speed.linear.x = 0.0
                        speed.angular.z = 0.0
                        self.goal_reached = True
                        self.arrival_time = self.get_clock().now().nanoseconds
                        self.approach_status = "scanning"
                else:
                    if angle_to_goal > 0.1 and angle_to_goal < math.pi and dist > 3.0:
                        print('Adjusting Angle - positive')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = TURN_SPEED
                    #elif angle_to_goal - odometry_pose_theta < -0.1 and angle_to_goal - odometry_pose_theta > -math.pi:
                    elif angle_to_goal  < -0.1 and angle_to_goal > -math.pi and dist > 3.0:
                        print('Adjusting Angle - negative')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = -TURN_SPEED    
                    elif angle_to_goal > 0.2 and angle_to_goal < math.pi:
                        print('Adjusting Angle - positive')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = TURN_SPEED
                    #elif angle_to_goal - odometry_pose_theta < -0.1 and angle_to_goal - odometry_pose_theta > -math.pi:
                    elif angle_to_goal  < -0.2 and angle_to_goal > -math.pi:
                        print('Adjusting Angle - negative')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = -TURN_SPEED
                    elif dist > 6.0:
                        print('Fast Approach')
                        speed.linear.x = HIGH_SPEED
                        speed.angular.z = 0.0
                    elif dist > 2.5:
                        print('Moderate Approach')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = 0.0
                    elif dist > 1.0:
                        print('Slow Approach')
                        speed.linear.x = SLOWEST_SPEED
                        speed.angular.z = 0.0
                    else:
                        print('Goal Reached!')
                        speed.linear.x = 0.0
                        speed.angular.z = 0.0
                        self.goal_reached = True
                        self.arrival_time = self.get_clock().now().nanoseconds
                        self.approach_status = "scanning"
                        scan_complete = False
            if self.approach_status == "scanning":
                wait_time = 30 * 10**9
                assert sys.version_info >= (3, 5)
                get_lock(__file__)

                # open port
                sp = usb_reader_open(RFID_READER_USB_PORT)

                if self.get_clock().now().nanoseconds < self.arrival_time + wait_time:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.approach_status = "scanning"
                    print("scanning", self.arrival_time + wait_time - self.get_clock().now().nanoseconds)
                    a = usb_cmd(sp, 'x')
                    if len(a) > 20:
                        lg.info(a.decode().strip('\r\n'))
                        print('TAG DETECTED!!', a)
                        self.approach_status = "disengaging"
                        usb_reader_close(sp)
                    else:
                        print('.')
                    
                else:
                    self.approach_status = "disengaging"
                    usb_reader_close(sp)
            if self.approach_status == "disengaging":
                if dist > 3.0:
                    print('Backed Away')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.goal_reached = False
                    self.approach_status = "complete"
                elif dist > 2.0:
                    print('Moderate Disengagement')
                    speed.linear.x = -MEDIUM_SPEED
                    speed.angular.z = 0.0
                elif dist > 0.0:
                    print('Slow Disengage')
                    speed.linear.x = -SLOWEST_SPEED
                    speed.angular.z = 0.0
                    
        self.velocity_publisher_.publish(speed)

    def publish_approach_status(self):
        self.penguin_status = PenguinApproachStatus()
        self.penguin_status.label = self.current_penguin_label
        self.penguin_status.status = self.approach_status
        self.approach_status_publisher_.publish(self.penguin_status)
    	
 
def main(args=None):
    rclpy.init(args=args)
    node = HuskyVelocityPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
  
if __name__ == '__main__':
  main()
