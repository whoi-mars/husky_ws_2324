#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import click
import math
import time
from rclpy.action import ActionClient
import transformations
import nav2_publish
import custom_nav_stack

from geographiclib.geodesic import Geodesic
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal   #Need to handle for movement part
from sensor_msgs.msg import NavSatFix

from pathlib import Path

def DMS_to_decimal_format(lat,long):
    # Check for degrees, minutes, seconds format and convert to decimal
    if ',' in lat:
      degrees, minutes, seconds = lat.split(',')
      degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
      if lat[0] == '-': # check for negative sign
        minutes = -minutes
        seconds = -seconds
      lat = degrees + minutes/60 + seconds/3600
    if ',' in long:
      degrees, minutes, seconds = long.split(',')
      degrees, minutes, seconds = float(degrees), float(minutes), float(seconds)
      if long[0] == '-': # check for negative sign
        minutes = -minutes
        seconds = -seconds
      long = degrees + minutes/60 + seconds/3600

    lat = float(lat)
    long = float(long)
    rclpy.logging.get_logger('DMS_to_decimal_format').info('Given GPS goal: lat %s, long %s.' % (lat, long))
    return lat, long

class GpsGoal(Node):
  def __init__(self):
    
    super().__init__('gps_goal')
    # rclpy.init_node('gps_goal')

    self.get_logger().info("Connecting to move_base...")
    # self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # self.move_base.wait_for_server()
    # self.get_logger().info("Connected.")

    self.create_subscription(PoseStamped, 'gps_goal_pose', self.gps_goal_pose_callback, 10) 
    self.create_subscription(NavSatFix, 'gps_goal_fix', self.gps_goal_fix_callback, 10)
    self.local_xy_origin_sub = self.create_subscription(PoseStamped, 'local_xy_origin', self.local_xy_origin_callback, 10)

    # self.origin_lat = 0  #JUST FOR TESTING
    # self.origin_long = 0 #JUST FOR TESTING
    # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
    # Get the lat long coordinates of our map frame's origin which must be publshed on topic /local_xy_origin. We use this to calculate our goal within the map frame.
    rclpy.logging.get_logger('get_origin_lat_long').info("Waiting for a message to initialize the origin GPS location...")
    # self.local_xy_origin_callback_called = 0
    # while self.local_xy_origin_callback_called != 1:
    #   pass
    #   rclpy.logging.get_logger('get_origin_lat_long').info("Waiting inside while loop")
    #   rclpy.logging.get_logger('get_origin_lat_long').info("self.local_xy_origin_callback_called: %s" % (self.local_xy_origin_callback_called))
    #   rclpy.spin(gpsGoal)
      # time.sleep(10000)
    

    # origin_pose = rclpy.wait_for_message('local_xy_origin', PoseStamped)
    # origin_lat = origin_pose.pose.position.y
    # origin_long = origin_pose.pose.position.x
    # self.get_logger().info('Inside get_origin_lat_long fn: Received origin: lat %s, long %s.' % (self.origin_lat, self.origin_long))
    # return origin_lat, origin_long
  



  def local_xy_origin_callback(self, msg):
    # self.local_xy_origin_callback_called = 1
    self.origin_lat = msg.pose.position.y
    self.origin_long = msg.pose.position.x
    # self.get_logger().info('Inside callback fn:  self.local_xy_origin_callback_called: %s.' % (self.local_xy_origin_callback_called))
    self.get_logger().info('Inside callback fn: Received origin: lat %s, long %s.' % (self.origin_lat, self.origin_long))
    self.destroy_subscription(self.local_xy_origin_sub)
    # return self.origin_lat, self.origin_long

  def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
    hypotenuse = distance = g['s12'] # access distance
    self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
    azimuth = g['azi1']
    self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
    # Convert azimuth to radians
    azimuth = math.radians(azimuth)
    x = adjacent = math.cos(azimuth) * hypotenuse
    y = opposite = math.sin(azimuth) * hypotenuse
    self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
    return x, y
  
  def do_gps_goal(self, goal_lat, goal_long, z=0, yaw=0, roll=0, pitch=0):
    # Calculate goal x and y in the frame_id given the frame's origin GPS and a goal GPS location
    x, y = self.calc_goal(self.origin_lat, self.origin_long, goal_lat, goal_long)
    
    
    # METHOD 1: Create move_base goal
    #self.publish_goal(x=x, y=y, z=z, yaw=yaw, roll=roll, pitch=pitch)

    #METHOD 2: Send NAV2 goal using nav2_publish.py
    # nav2_publish.main([x, y])

    #METHOD 3: Own navigation stack
    custom_nav_stack.main([self.origin_lat, self.origin_long, goal_lat, goal_long, x, y])
    

  def gps_goal_pose_callback(self, data):
    lat = data.pose.position.y
    long = data.pose.position.x
    z = data.pose.position.z
    euler = transformations.euler_from_quaternion(data.pose.orientation)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    self.do_gps_goal(lat, long, z=z, yaw=yaw, roll=roll, pitch=pitch)

  def gps_goal_fix_callback(self, data):
    self.do_gps_goal(data.latitude, data.longitude)

  # def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0):
  #   # Create move_base goal
  #   goal = MoveBaseGoal()
  #   goal.target_pose.header.frame_id = rclpy.get_param('~frame_id','map')
  #   goal.target_pose.pose.position.x = x
  #   goal.target_pose.pose.position.y = y
  #   goal.target_pose.pose.position.z =  z
  #   quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
  #   goal.target_pose.pose.orientation.x = quaternion[0]
  #   goal.target_pose.pose.orientation.y = quaternion[1]
  #   goal.target_pose.pose.orientation.z = quaternion[2]
  #   goal.target_pose.pose.orientation.w = quaternion[3]
  #   self.get_logger().info('Executing move_base goal to position (x,y) %s, %s, with %s degrees yaw.' %
  #           (x, y, yaw))
  #   self.get_logger().info("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

  #   # Send goal
  #   self.move_base.send_goal(goal)
  #   self.get_logger().info('Inital goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
  #   status = self.move_base.get_goal_status_text()
  #   if status:
  #     self.get_logger().info(status)

  #   # Wait for goal result
  #   self.move_base.wait_for_result()
  #   self.get_logger().info('Final goal status: %s' % GoalStatus.to_string(self.move_base.get_state()))
  #   status = self.move_base.get_goal_status_text()
  #   if status:
  #     self.get_logger().info(status)

@click.command()
# @click.option('--lat', prompt='Latitude', help='Latitude')
# @click.option('--long', prompt='Longitude', help='Longitude')
@click.option('--roll', '-r', help='Set target roll for goal', default=0.0)
@click.option('--pitch', '-p', help='Set target pitch for goal', default=0.0)
@click.option('--yaw', '-y', help='Set target yaw for goal', default=0.0)
# def cli_main(lat, long, roll, pitch, yaw, args=None):
def cli_main(roll, pitch, yaw, args=None):
  """Send goal to move_base given latitude and longitude

  \b
  Two usage formats:
  gps_goal.py --lat 43.658 --long -79.379 # decimal format
  gps_goal.py --lat 43,39,31 --long -79,22,45 # DMS format
  """
  path = Path(__file__).parent / "./destination_lat_long.txt"
  f = open(path, "r")
  Lines = f.readlines()
  lat_array =[]
  long_array = []
  count = 0
  # Strips the newline character
  for line in Lines:
      if not (Lines[count].startswith('#') or Lines[count].startswith('\n') or Lines[count].startswith(' ')) :
        print("Line{}: {}".format(count, line.rstrip('\n').strip('')))
        lat_array.append(Lines[count].rstrip('\n').split(' ')[0])
        long_array.append(Lines[count].rstrip('\n').split(' ')[1])
      count += 1

  print(lat_array, long_array)
  rclpy.init(args=args)
  gpsGoal = GpsGoal();
  rclpy.spin_once(gpsGoal)

  for i in range(len(lat_array)):
    lat = lat_array[i]
    long = long_array[i]
    # Check for degrees, minutes, seconds format and convert to decimal
    lat, long = DMS_to_decimal_format(lat, long)
    gpsGoal.do_gps_goal(lat, long, roll=roll, pitch=pitch, yaw=yaw)


def ros_main():
  gpsGoal = GpsGoal();
  rclpy.spin()

if __name__ == '__main__':
  cli_main()
