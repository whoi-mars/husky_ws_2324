#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SOURCE: https://github.com/ros-planning/navigation2/issues/2283


from logging.handlers import BaseRotatingHandler
from threading import local
import time
import math
import subprocess


from geometry_msgs.msg import Pose, PoseStamped, Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
import sys
import rclpy
import utm
import numpy as np
import pandas as pd
import random
import open3d as o3d
import ros2_numpy as rnp
from transformations import euler_from_quaternion
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import pyransac3d as pyrsc

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from geographiclib.geodesic import Geodesic

from pathlib import Path
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import socket
import struct
import errno
import sys

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432  # The port used by the server


#ASSUMPTIONS: 1) IMU 0 orientation is TRUE NORTH | 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)


class CustomNavigator(Node):
    def __init__(self):
        super().__init__(node_name='custom_navigator')
        
        # self.fig = plt.figure()
        # self.ax = plt.axes(projection='3d')

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, PORT))
        self.s.setblocking(False)
        print("Socket connected")
        time.sleep(3)

        self.fig = plt.figure()
        self.fig2 = plt.figure(2)


        # fig, self.axs = plt.subplots(2, 2,)
        
        # self.axs[0, 0].set_title('Axis [0, 0]')
        # self.axs[0, 1].set_title('Axis [0, 1]')
        # self.axs[1, 0].set_title('Axis [1, 0]')
        # self.axs[1, 1].set_title('Axis [1, 1]')

        
        self.gps_callback_done = 0
        self.gps_subscription = self.create_subscription(NavSatFix,'gps/data',self.gps_callback,10)
        self.gps_subscription  # prevent unused variable warning

        self.imu_subscription = self.create_subscription(Imu,'imu/data',self.imu_callback,10)
        self.imu_subscription  # prevent unused variable warning

        self.odom_subscription = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.odom_subscription  # prevent unused variable warning

        # self.pc_subscription = self.create_subscription(PointCloud2,'velodyne_points',self.pc_callback,10)
        # self.pc_subscription  # prevent unused variable warning

        
        path = str(Path(__file__).parent / "./destination_lat_long.txt")
        path = path.replace("install", "src")
        path = path.replace("lib/custom_nav_stack_pkg", "scripts")

        f = open(path, "r")
        Lines = f.readlines()
        self.lat_array =[]
        self.long_array = []
        count = 0
        # Strips the newline character
        for line in Lines:
            if not (Lines[count].startswith('#') or Lines[count].startswith('\n') or Lines[count].startswith(' ')) :
              print("Line{}: {}".format(count, line.rstrip('\n').strip('')))
              self.lat_array.append(Lines[count].rstrip('\n').split(' ')[0])
              self.long_array.append(Lines[count].rstrip('\n').split(' ')[1])
            count += 1

        print(self.lat_array, self.long_array)

        self.velocity_publisher_ = self.create_publisher(Twist, 'husky_velocity_controller/cmd_vel_unstamped', 10)
        timer_period = 1/100  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        print("Entered in class")

        for i in range(len(self.lat_array)):
            # Check for degrees, minutes, seconds format and convert to decimal
            self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[i], self.long_array[i])
        
        global goal_number
        goal_number = 0
        self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])

        # self.current_lat = math.radians(argv[0])
        # self.current_long = math.radians(argv[1])
        # self.dest_lat = math.radians(argv[2])
        # self.dest_long = math.radians(argv[3])
        # self.delta_long = dest_long - current_long

        # self.current_lat = math.radians(argv[0])
        # self.current_long = math.radians(argv[1])
        # self.dest_lat = math.radians(argv[2])
        # self.dest_long = math.radians(argv[3])
        # self.delta_long = dest_long - current_long

        # Reference: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/
        
        
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # self.target = 90
        # self.target2 = -90 
        self.kp = 0.8
        # self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        self.mag_declination = math.radians(-4.23) # 4.23° W at 0 lat/ long
        self.imu_heading_offset = math.radians(4.235193576741463 - 90) #At robot's initial spawn position, IMU provides ~ 4.23° heading  which is towards West. Need to account for this offset. In practice, it shouldn't matter as heading will be calculated from magnetometer readings which are wrt real directions.
        
        
        # time.sleep(5)
        
        # Twist is a datatype for velocity
        self.move_cmd = Twist()

        # self.gps_sub = self.create_subscription(MagneticField, '/mag', self.mag_callback)  #To be used on real robot as we couldn't get mag in gazebo
        #self.mag_heading = math.atan2(mag_y, magx)

        # self.mag_north_heading = math.radians(10) # 0 means that the magnetometer is actually pointing towards Magnetic North; +10 means that you are 10° towards East of magnetic north; i.e. need to turn 10° towards left to reach magnetic north
        
        #Reference: Calculate magnetic declination (measured wrt True North): https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination; 2022-08-26 	14.87° W  ± 0.40°  changing by  0.06° W per year
        # By convention, declination is positive when magnetic north is east of true north, and negative when it is to the west. https://upload.wikimedia.org/wikipedia/commons/thumb/c/c2/Magnetic_declination.svg/330px-Magnetic_declination.svg.png
        # self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        # self.true_heading = self.mag_north_heading + self.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North

        # self.to_rotate = bearing - self.true_heading
        # self.direction = 'ccw' if self.to_rotate < 0 else 'cw'
        # print(self.direction)
        # Publisher to send velocity commands to robot
        

        # if self.true_heading != 0:
        #     self.rotate(self.true_heading, self.direction)
        # elif self.true_heading == 0:
        #     print("Robot already aligned towards goal")

    def DMS_to_decimal_format(self, lat, long):
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
    
    def imu_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print("Entered IMU callback")
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list) #Result in radians. -180 < Roll < 180
        self.mag_north_heading = -self.roll + self.imu_heading_offset #ASSUMPTION 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)
        # print("IMU heading: ", math.degrees(self.mag_north_heading))
        # print("Finished IMU callback")

    def gps_callback(self, msg):
        self.gps_callback_done = 1
        self.current_lat = msg.latitude
        self.current_long = msg.longitude
        self.delta_long = self.dest_long - self.current_long
        X = math.cos(self.dest_lat) * math.sin(self.delta_long)
        Y = (math.cos(self.current_lat) * math.sin(self.dest_lat)) - (math.sin(self.current_lat) * math.cos(self.dest_lat) * math.cos(self.delta_long))
        global bearing
        bearing = math.atan2(X, Y)
        rclpy.logging.get_logger('gps_callback').info('Bearing (degree):  %s' % math.degrees(bearing))        # print("Entered IMU callback")
        # print("Bearing in radian: ", bearing)
        # print("Bearing in degree: ", math.degrees(bearing)) #If -45° , means need to point robot to 45° left/ west of True North

         
        

    def odom_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # print("Entered ODOM callback")
        self.current_odom_x = msg.pose.pose.position.x
        # print("Odom x: ", self.current_odom_x)
        

    def pc_callback(self, msg):
        # msg.__class = PointCloud2
        #offset_sorted = {f.offset: f for f in msg.fields}
        #msg.fields = [f for(_, f) in sorted(offset_sorted.items())]
        data = rnp.point_cloud2.pointcloud2_to_xyz_array(msg,remove_nans=True) #Reading xyz values from PointCloud2 message
        #pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
        data = np.append(data, np.arange(data.shape[0]).reshape(-1, 1), axis=1) #Appending index to the 4th column
        
        self.ax = self.fig.add_subplot(221, projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_title('Original Point Cloud, size: %.i' % data.shape[0])
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.set_zlim(-1.5,1.2)
        self.ax.scatter(data[:,0], data[:,1], data[:,2]) #Plotting original cloud

        self.ax2 = self.fig2.add_subplot(221, projection='3d')
        self.ax2.set_xlabel('X axis')
        self.ax2.set_ylabel('Y axis')
        self.ax2.set_zlabel('Z axis')
        self.ax2.set_title('Original Point Cloud, size: %.i' % data.shape[0])
        self.ax2.set_xlim(-5,5)
        self.ax2.set_ylim(-5,5)
        self.ax2.set_zlim(-1.5,1.2)
        self.ax2.scatter(data[:,0], data[:,1], data[:,2]) #Plotting original cloud



        
        #Filtering points based on height
        mean = np.mean(data[:,2])
        sd = np.std(data[:,2])
        data_ground = data[(abs(data[:,2]) > 0.8) & (abs(data[:,2]) < 1.2)] #Lidar is ~ 1 m above from ground. Points with z lying between 0.8 and 1.2 m are considered ground
        # data_ground = data[(data[:,2] < mean + 1.5*sd) & (data[:,2] > mean - 1.5*sd)] #Filtering ground points based on height assumption
        # data_wo_ground = data[(data[:,2] > mean + 1.5*sd) | (data[:,2] < mean - 1.5*sd)]
        data_wo_ground = np.delete(data, data_ground[:,3].astype(int), axis=0)[:,:3]
        self.ax = self.fig.add_subplot(222, projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_title('Ground Points filtered by height, size: %.i' % data_ground.shape[0])
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.set_zlim(-1.5,1.2)
        self.ax.scatter(data_ground[:,0], data_ground[:,1], data_ground[:,2]) #Plotting ground points filtered based on height

        self.ax2 = self.fig2.add_subplot(222, projection='3d')
        self.ax2.set_xlabel('X axis')
        self.ax2.set_ylabel('Y axis')
        self.ax2.set_zlabel('Z axis')
        self.ax2.set_title('Non-ground points filtered by height, size: %.i' % data_wo_ground.shape[0])
        self.ax2.set_xlim(-5,5)
        self.ax2.set_ylim(-5,5)
        self.ax2.set_zlim(-1.5,1.2)
        self.ax2.scatter(data_wo_ground[:,0], data_wo_ground[:,1], data_wo_ground[:,2]) #Plotting non-ground points filtered based on height
        
        #PCA implementation
        max_z, min_z = np.max(data_ground[:, 2]), np.min(data_ground[:, 2])
        data_ground[:, 2] = (data_ground[:, 2] - min_z)/(max_z - min_z)
        covariance = np.cov(data_ground[:, :3].T)
        eigen_values, eigen_vectors =  np.linalg.eig(np.matrix(covariance))
        # normal_vector = eigen_vectors[np.argmin(eigen_values)] #For plane consisting obstacles
        normal_vector = eigen_vectors[np.argmax(eigen_values)] #For pure ground plane
        projection = normal_vector.dot(data_ground[:, :3].T)
        # ground_mask = np.abs(projection) < 0.4 #For plane consisting obstacles
        ground_mask = np.abs(projection) > 0.010 #For pure ground plane
        data_ground = np.asarray([data_ground[index[1]] for index, a in np.ndenumerate(ground_mask) if a == True])
        data_ground[:, 2] = data_ground[:, 2] * (max_z - min_z) + min_z
        data_wo_ground = np.delete(data, data_ground[:,3].astype(int), axis=0)[:,:3]
        print("PCA output: ",data.shape, data_ground.shape, data_wo_ground.shape)
        # print(data_ground[:,3])
        
        self.ax = self.fig.add_subplot(223, projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_title('PCA ground points, size: %.i' % data_ground.shape[0])
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.set_zlim(-1.5,1.2)
        self.ax.scatter(data_ground[:,0], data_ground[:,1], data_ground[:,2]) #Plotting ground points filtered through PCA

        self.ax2 = self.fig2.add_subplot(223, projection='3d')
        self.ax2.set_xlabel('X axis')
        self.ax2.set_ylabel('Y axis')
        self.ax2.set_zlabel('Z axis')
        self.ax2.set_title('PCA non-ground points, size: %.i' % data_wo_ground.shape[0])
        self.ax2.set_xlim(-5,5)
        self.ax2.set_ylim(-5,5)
        self.ax2.set_zlim(-1.5,1.2)
        self.ax2.scatter(data_wo_ground[:,0], data_wo_ground[:,1], data_wo_ground[:,2]) #Plotting non-ground points filtered through PCA
        
        #RANSAC implementation
        # random.seed()
        # inliers_index_array = [] #Contains inlier global index for all the iterations
        # max_iterations = 6
        # threshold = 0.4 #Distance threshold in metre
        # score_array = [] #Contains score for all the iterations


        # for j in range(max_iterations):
        #     local_random_index = []
        #     global_random_index = np.random.choice(data_ground[:,3].astype(int), size=3, replace=False) #Picking 3 points at random without replacement/ repeatition
        #     # print("Global indexes are: ", global_random_index)
        #     for i in global_random_index:
        #         # print(i)
        #         # print(np.where(data_ground[:,3].astype(int) == i)[0][0])
        #         local_random_index.append(np.where(data_ground[:,3].astype(int) == i)[0][0])

        #     # print("Local indexes are: ",local_random_index)
        #     print("Random points global index as per original xyz data: ",global_random_index)
        #     print("Random points local index in data_ground array: ", local_random_index)
        #     # print("Global: ", data[global_random_index,:],  " \nLocal: ", data_ground[local_random_index, :])
        #     # print(data_ground[local_random_index, :3])

        #     [x1, y1, z1], [x2, y2, z2], [x3, y3, z3] = data_ground[local_random_index, :3]

        #     # Plane Equation --> ax + by + cz + d = 0
        #     # Value of Constants for inlier plane
        #     a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)
        #     b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1)
        #     c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)
        #     d = -(a*x1 + b*y1 + c*z1)

        #     inliers_index_array = np.append(inliers_index_array, global_random_index)
        #     score = 3 #Including the 3 random points as well for score calculation

        #     #Find distance of all other points from the plane
        #     for i in range(data_ground.shape[0]): #Check distance of all other points from the plane. Does not include the last number. range starts from 0
        #         if i not in local_random_index:
        #             x4, y4, z4 = data_ground[i, :3]
        #             distance = ((a*x4) + (b*y4) + (c*z4) + d) / math.sqrt(a*a + b*b + c*c) 
        #             # print(x4, y4, z4)

        #             if distance < threshold: #It means the point is a ground point/ inlier
        #                 # print("Inlier global index: ", np.where(data[:,3].astype(int) == i)[0][0])
        #                 inliers_index_array = np.append(inliers_index_array, np.where(data[:,3].astype(int) == data_ground[i,3])[0][0])  #Need to store global index of x4, y4, z4
        #                 score+= 1
        #     score_array.append(score)
        # print("Inliers arrays all iteration, size: ", inliers_index_array.shape)
        # #Choosing iteration with maximum score

        # max_score = np.amax(score_array)
        # max_score_index = np.argmax(score_array)
        # print("Scores for iterations are: ", score_array)
        # print("Maximum score is: ", max_score, "| Corresponding index is: ", max_score_index)

        # if max_score_index == 0:
        #     start_index = 0
        # else:
        #     print("Sum till max score index: ",sum(score_array[:max_score_index]))
        #     start_index = sum(score_array[:max_score_index]) - 1
        # end_index = start_index + score_array[max_score_index]
        # print("Start Index: ", start_index, "End Index: ", end_index)
        # print("Inlier points global index: ", inliers_index_array[start_index : end_index])

        
        # data_ground = data[inliers_index_array[start_index : end_index].astype(int)]
        # data_wo_ground = np.delete(data, data_ground[:,3].astype(int), axis=0)[:,:3]
        # print("RANSAC output: ",data.shape, data_ground.shape, data_wo_ground.shape)
        # self.ax = self.fig.add_subplot(222, projection='3d')
        # self.ax.scatter(data[:,0], data[:,1], data[:,2])

        #Example 1 - Planar RANSAC¶ : Reference: https://leomariga.github.io/pyRANSAC-3D/api-documentation/plane/
        points = np.array([data_ground[:,0],data_ground[:,1],data_ground[:,2]]).T
        print("Points shape: ", points.shape)
        plane1 = pyrsc.Plane()
        best_eq, inliers_local_index_array = plane1.fit(points, thresh=0.02) #best_inliers contains the local index of inliers present inside data_ground array
        inliers_global_index_array = data_ground[inliers_local_index_array,3]
        
                
        print("Points shape: ", points.shape, "Best eq: ", best_eq, "Best inliers shape: ", inliers_global_index_array.shape, "Inliers index array shape: ", inliers_global_index_array.shape)
        data_ground = data[inliers_global_index_array.astype(int)]
        data_wo_ground = np.delete(data, data_ground[:,3].astype(int), axis=0)[:,:3]
        print("RANSAC output: ",data.shape, data_ground.shape, data_wo_ground.shape)

        self.ax = self.fig.add_subplot(224, projection='3d')
        self.ax.set_xlabel('X axis')
        self.ax.set_ylabel('Y axis')
        self.ax.set_zlabel('Z axis')
        self.ax.set_title('RANSAC ground points, size: %.i' % data_ground.shape[0])
        self.ax.scatter(data_ground[:,0], data_ground[:,1], data_ground[:,2]) #Plotting ground points filtered through RANSAC
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.set_zlim(-1.5,1.2)

        self.ax2 = self.fig2.add_subplot(224, projection='3d')
        self.ax2.set_xlabel('X axis')
        self.ax2.set_ylabel('Y axis')
        self.ax2.set_zlabel('Z axis')
        self.ax2.set_title('RANSAC non-ground points, size: %.i' % data_wo_ground.shape[0])
        self.ax2.set_xlim(-5,5)
        self.ax2.set_ylim(-5,5)
        self.ax2.set_zlim(-1.5,1.2)
        self.ax2.scatter(data_wo_ground[:,0], data_wo_ground[:,1], data_wo_ground[:,2]) #Plotting non-ground points filtered through RANSAC

        if data_wo_ground.shape[0] != 0: #Means there is no non-ground points/ obstacle
            distance_array = np.zeros(shape=(data_wo_ground.shape[0],1))
            for i in range(data_wo_ground.shape[0]): 
                distance = math.sqrt(data_wo_ground[i,0]**2 + data_wo_ground[i,1]**2 + data_wo_ground[i,2]**2)
                #print(distance)
                distance_array[i,0] = distance
            # print(np.amin(distance_array))
            print("Minimum distance from VLP-16 is:", np.amin(distance_array), ", index is: ", np.argmin(distance_array), "and XYZ point is: ", data_wo_ground[np.argmin(distance_array)])
        
        # plt.show()
        #print(distance_array.shape)
        #print(data[1,0])
        print("Listening END. Size is: " + str(data.shape))

        # rclpy.logging.get_logger('gps_callback').info('Bearing (degree):  %s' % math.degrees(bearing))        # print("Entered IMU callback")
        

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
        azimuth = g['azi1']
        # self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        # self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
        return distance
    
    def move_forward(self, distance_to_move): # If distance_to_move='until_obstacle', then robot will move forward indefinitely until an obstacle is detected, otherwise give distance in meters
        if distance_to_move > 1:
            print("Need to move: ", distance_to_move)
            self.move_cmd.linear.x = 0.2
            self.move_cmd.angular.z = 0.0
            self.velocity_publisher_.publish(self.move_cmd)
        elif(distance_to_move < 1):
            print("Destination reached :)")
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.velocity_publisher_.publish(self.move_cmd)
            global goal_number
            goal_number+=1
            if goal_number < len(self.lat_array):
                self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])

    def timer_callback(self):
        # print("Entered timer callback")
        
        if self.gps_callback_done == 1:
            #First try to align robot to desired heading and then move in that direction
            global bearing
            # rclpy.logging.get_logger('timer_callback').info('Entered IMU/ timer_callback') 


            self.true_heading = self.mag_north_heading + self.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
            print("Current heading: ", math.degrees(self.true_heading))
            # Calculation to get true heading within -180 to +180 degrees range
            if self.true_heading < math.radians(-180):
                self.true_heading = math.radians(180 - (abs(math.degrees(self.true_heading)) - 180))
            elif self.true_heading > math.radians(180):
                self.true_heading = -(math.radians(180) - (self.true_heading - math.radians(180)))
            print("Current heading after 180 adjustment: ", math.degrees(self.true_heading))

            # Calculations to find the angle to rotate the robot in order to align with the required heading (true heading)
            if bearing < 0:
                if (self.true_heading < bearing and self.true_heading > math.radians(-180)) or (self.true_heading > (math.radians(180) - abs(bearing)) and self.true_heading < math.radians(180)):
                    self.direction = 'cw' #Original
                    if self.true_heading < 0:
                        self.to_rotate = abs(bearing - self.true_heading)
                    else:
                        self.to_rotate = math.radians(180 - math.degrees(self.true_heading) + 180 - math.degrees(abs(bearing)))
                elif ((self.true_heading < 0 and self.true_heading > bearing) or (self.true_heading > 0 and self.true_heading < math.radians(180 - math.degrees(abs(bearing))))):
                    self.direction = 'ccw' #Original
                    if self.true_heading < 0:
                        self.to_rotate = abs(bearing - self.true_heading)
                    else:
                        self.to_rotate = self.true_heading + abs(bearing)
            elif bearing > 0:
                if ((self.true_heading < 0 and self.true_heading > -math.radians(180 - math.degrees(bearing))) or (self.true_heading > 0 and self.true_heading < bearing)):
                    self.direction = 'cw' #Original
                    if self.true_heading < 0:
                        self.to_rotate = abs(self.true_heading) + bearing
                    else:
                        self.to_rotate = abs(bearing - self.true_heading)
                elif ((self.true_heading > bearing and self.true_heading < math.radians(180)) or (self.true_heading < -math.radians(180 - math.degrees(bearing)) and self.true_heading > math.radians(-180))):
                    self.direction = 'ccw' #Original
                    
                    if self.true_heading < 0:
                        self.to_rotate = math.radians(180 - abs(math.degrees(self.true_heading)) + 180 - math.degrees(bearing))
                    else:
                        self.to_rotate = abs(bearing - self.true_heading)


            rclpy.logging.get_logger('timer_callback').info('Need to rotate robot by (degree):  %s ° in %s ' % (math.degrees(self.to_rotate), self.direction))        # print("Entered IMU callback")
            # print("Need to rotate robot by", math.degrees(self.to_rotate), "° in ", self.direction)
            # self.to_rotate = bearing - self.true_heading
            # self.direction = 'ccw' if self.to_rotate < 0 else 'cw'
            # print(math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw))
            # print(self.roll, self.pitch, self.yaw)
            # print(math.degrees(self.roll)) #Roll is providing the current angular rotation surprisingly

            # Reference: https://stackoverflow.com/questions/16745409/what-does-pythons-socket-recv-return-for-non-blocking-sockets-if-no-data-is-r
            
            global obs_corner_angle_rad
            try: 
                while(1==1): 
                    obs_corner_angle_rad = struct.unpack('f', self.s.recv(4))[0]
                    print("Received data over socket")
                    # obs_corner_angle_rad = data
                    print("################################### OLD CORNER DETECTED AT ANGLE DEGREE: ", obs_corner_angle_rad*180/math.pi, "###########################")
            except socket.error as e:
                err = e.args[0]
                if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                    print("No data available")
                    print("################################### LATEST CORNER DETECTED AT ANGLE DEGREE: ", obs_corner_angle_rad*180/math.pi, "###########################")
                    if obs_corner_angle_rad != 456.7799987792969:
                        if obs_corner_angle_rad >= 0: #means obstacle on left, turn towards right or CW
                            self.direction = 'cw'
                            self.to_rotate = math.radians(20)
                        else:
                            self.direction = 'ccw'
                            self.to_rotate = math.radians(20)
                else:
                    # a "real" error occurred
                    print("Real error occured: ", e)
                    sys.exit(1)  



            
            
            if abs(self.to_rotate) > 0.06:
                self.rotate(self.to_rotate, self.direction)
            elif self.to_rotate < 0.06: #means no obstacle in FOV and heading is also the desired one
                print("Robot already aligned towards goal")
                print("Finished timer callback")
                self.distance_to_move = self.calc_goal(self.current_lat, self.current_long, self.dest_lat, self.dest_long)
                self.move_forward(self.distance_to_move)
                # self.destroy_timer(self.timer)

            


            

    def rotate(self,target_angle_rad,direction):
        print("Entered rotate function")
        global bearing
        if(direction == 'CCW' or direction == 'ccw'):
                multiplier = 1  #Positive velocity means anti-clockwise (CCW) rotation (If we rotate in actual CCW direction, robot moves away from the bearing.. its a workaround)
                print("CCW")
        else:
                multiplier = -1  #Negative velocity means clockwise (CW) rotation
                print("CW")
        #print("Reached 1")
        # target_angle_rad = multiplier * target_angle_deg*math.pi/180
        #print("Angular z is: ",self.move_cmd.angular.z)
        # publish the velocit
        rotation_complete = 0
        print("Bearing (in degree): ", math.degrees(bearing))
        print("True heading (in degree): ", math.degrees(self.true_heading))
        # print("DEGREE: Need_to_rotate={} current heading:{}", math.degrees(target_angle_rad), math.degrees(self.true_heading))
        print("RADIAN: Need_to_rotate by: ", math.degrees(target_angle_rad), "in ", direction)
        
        # if abs(target_angle_rad - self.roll) < 0.06:
            # rotation_complete = 1
            # print("Met the target rotation approximately")
            # self.move_cmd.angular.z = 0.0
            # self.velocity_publisher_.publish(self.move_cmd)
            # self.destroy_timer(self.timer)
            # # time.sleep(8)
            # return None
        # else:
        print("Reached 2")
        # print("target={} current:{}", target_angle_rad,self.roll)
        # self.move_cmd.angular.z = multiplier * (self.kp * (abs(bearing - self.true_heading) + 0.3))
        # self.move_cmd.angular.z = multiplier * (self.kp * (abs(target_angle_rad) + 0.3))
        # self.move_cmd.linear.x = abs(self.move_cmd.angular.z / 4)
        self.move_cmd.angular.z = 0.2
        self.move_cmd.linear.x = 0.1
        # self.move_cmd.angular.z = multiplier * 0.5
        # print(self.move_cmd.angular.z)
        self.velocity_publisher_.publish(self.move_cmd)
        time.sleep(1)
        print("Finished single execution of rotate function")
            

def main(argv=sys.argv[1:]):

    # print(argv[0], argv[1])
    rclpy.init()


    
    custom_navigator = CustomNavigator()
    rclpy.spin(custom_navigator)
    custom_navigator.s.close()
    print("Closed socket on client side")

    
    

if __name__ == '__main__':
  main()
