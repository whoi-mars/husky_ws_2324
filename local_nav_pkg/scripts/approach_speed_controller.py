#! /usr/bin/env python3

# Script to handle the approach of each penguin, including the firing of the RFID antenna and disengagement.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from local_nav_pkg.msg import PenguinApproachStatus
import math

import datetime
import logging
import sys
from logging.handlers import TimedRotatingFileHandler
from rfid_reader.utils import get_lock
from getmac import get_mac_address
from rfid_reader.utils_usb import usb_reader_open, usb_reader_close, usb_cmd

# RFID Reader Initialization
ANTENNA_ON = True

RFID_READER_USB_PORTS = ['dev/rfid_port','/dev/ttyUSB0','/dev/ttyUSB1','/dev/ttyUSB2','/dev/ttyUSB3']

mac = get_mac_address().replace(':', "-")
now = datetime.datetime.now().strftime('%Y%m%d')
path = '/home/administrator/husky_ws2/src/local_nav_pkg/scripts/rfid_reader/logs/{}_rfid_daemon_{}.log'.format(now, mac)
lg = logging.getLogger(path)
lg.setLevel(logging.DEBUG)
han = TimedRotatingFileHandler(filename=path, when='midnight', interval=1)
fmt = logging.Formatter("%(asctime)s | %(message)s", '%Y%m%d-%H%M%S')
han.setFormatter(fmt)
lg.addHandler(han)

# Speed Setpoints
SLOWEST_SPEED = 0.01
MEDIUM_SPEED = 0.025
HIGH_SPEED = 0.1

TURN_SPEED = 0.025

# Uncomment for Test Speeds
# SLOWEST_SPEED = 0.04
# MEDIUM_SPEED = 0.2
# HIGH_SPEED = .4

# TURN_SPEED = .2

# Distance Setpoints
SCANNING_DISTANCE = 1.0
SLOW_APPROACH_ZONE = SCANNING_DISTANCE
MEDIUM_APPROACH_ZONE = 2.0
FAST_APPROACH_ZONE = 4.0

SCAN_DISTANCE_ANGLE_THRESHOLD = 0.75 # radians
SLOW_DISTANCE_ANGLE_THRESHOLD = 0.2
MEDIUM_DISTANCE_ANGLE_THRESHOLD = 0.1

SLOW_DISENGAGE_ZONE = 2.0
DISENGAGE_DISTANCE = 3.0


class HuskyVelocityPublisherNode(Node):

    def __init__(self):
        super().__init__('husky_velocity_publisher')
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, 'local_goal_pose', self.goal_pose_callback, 10)
        self.status_subscriber = self.create_subscription(PenguinApproachStatus, 'penguin_approach_status', self.approach_status_callback, 10)

        self.velocity_publisher_ = self.create_publisher(Twist, 'approach_vel', 10)
        self.approach_status_publisher_ = self.create_publisher(PenguinApproachStatus, 'penguin_approach_status', 10)

        self.velocity_timer_ = self.create_timer(.05, self.publish_husky_velocity)

        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0

        self.goal_reached = False
        self.arrival_time = 0
        self.current_penguin_label = str(0)
        self.approach_status = "pending"
        
    # Get existing goal pose for local nav    
    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y

    # Get existing approach status       
    def approach_status_callback(self, msg):
        self.current_penguin_label = msg.label
        self.approach_status = msg.status

    # Approach velocity calculation and publisher, also publishes approach status and starts the rfid antenna
    def publish_husky_velocity(self):
        # Distance to goal is currently just the distance in X
        dist = self.goal_pose_x
        speed = Twist()

        angle_to_goal = math.atan2(self.goal_pose_y, self.goal_pose_x)
        print('Angle to goal: ', round(angle_to_goal,2))
        
        #print('Distance to goal in x: ', round(x_dist,2),'Distance to goal in y: ', round(y_dist,2))
        print('Distance to goal in x: ', round(dist,2), self.approach_status)        
        
        # If there is no goal (label is zero) set velocity to zero
        if self.current_penguin_label == str(0):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        # If there is a goal, engage it
        else:
            if self.approach_status == "pending":
                self.approach_status = "engaging"
            if self.approach_status == "engaging":
                # If the robot is very close (under scanning distance) check the angle, if it is very sharp back up and try again	
                if dist < SCANNING_DISTANCE:
                    if abs(angle_to_goal) > SCAN_DISTANCE_ANGLE_THRESHOLD:
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
                        assert sys.version_info >= (3, 5)
                        get_lock(__file__)

                        # open port
                        if ANTENNA_ON:
                            sp = usb_reader_open(RFID_READER_USB_PORTS)
                else:
                    # Check the angle based on the distance from the target and turn if needed
                    if angle_to_goal > SLOW_DISTANCE_ANGLE_THRESHOLD and angle_to_goal < math.pi and dist > MEDIUM_APPROACH_ZONE:
                        print('Adjusting Angle - positive')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = TURN_SPEED
                    elif angle_to_goal  < -SLOW_DISTANCE_ANGLE_THRESHOLD and angle_to_goal > -math.pi and dist > MEDIUM_APPROACH_ZONE:
                        print('Adjusting Angle - negative')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = -TURN_SPEED    
                    elif angle_to_goal > MEDIUM_DISTANCE_ANGLE_THRESHOLD and angle_to_goal < math.pi:
                        print('Adjusting Angle - positive')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = TURN_SPEED
                    elif angle_to_goal  < -MEDIUM_DISTANCE_ANGLE_THRESHOLD and angle_to_goal > -math.pi:
                        print('Adjusting Angle - negative')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = -TURN_SPEED
                    # If angle is good, approach with speed dependent on distance to target
                    elif dist > FAST_APPROACH_ZONE:
                        print('Fast Approach')
                        speed.linear.x = HIGH_SPEED
                        speed.angular.z = 0.0
                    elif dist > MEDIUM_APPROACH_ZONE:
                        print('Moderate Approach')
                        speed.linear.x = MEDIUM_SPEED
                        speed.angular.z = 0.0
                    elif dist > SLOW_APPROACH_ZONE:
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
                        assert sys.version_info >= (3, 5)
                        get_lock(__file__)

                        # open port
                        if ANTENNA_ON:
                            sp = usb_reader_open(RFID_READER_USB_PORTS)
                                
            if self.approach_status == "scanning":
                # Wait for 30 seconds, trying to scan
                wait_time = 30 * 10**9        

                if self.get_clock().now().nanoseconds < self.arrival_time + wait_time:
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.approach_status = "scanning"
                    print("scanning", self.arrival_time + wait_time - self.get_clock().now().nanoseconds)
                    if ANTENNA_ON:
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
                    if ANTENNA_ON:
                        usb_reader_close(sp)
            
            if self.approach_status == "disengaging":
                if dist > DISENGAGE_DISTANCE:
                    print('Backed Away')
                    speed.linear.x = 0.0
                    speed.angular.z = 0.0
                    self.goal_reached = False
                    self.approach_status = "complete"
                elif dist > SLOW_DISENGAGE_ZONE:
                    print('Moderate Disengagement')
                    speed.linear.x = -MEDIUM_SPEED
                    speed.angular.z = 0.0
                elif dist > 0.0:
                    print('Slow Disengage')
                    speed.linear.x = -SLOWEST_SPEED
                    speed.angular.z = 0.0
        
        # Publish the velocity            
        self.velocity_publisher_.publish(speed)
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
