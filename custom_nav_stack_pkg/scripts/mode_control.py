#! /usr/bin/env python3

__author__ = "Dev Vaibhav"
__credits__ = ["Dev Vaibhav"]
__maintainer__ = "Dev Vaibhav"
__email__ = "dev.vaibhav@whoi.edu"

import logging
from statistics import mode
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy, NavSatFix, Imu, MagneticField
from geometry_msgs.msg import Twist, Vector3

from geographiclib.geodesic import Geodesic


import message_filters
from pathlib import Path
import math
from transformations import euler_from_quaternion
import time
import numpy as np
import array as arr
import statistics
import subprocess
from signal import SIGINT

import os
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

logger = logging.getLogger("my_logger")

class ModeControl(Node):

    def __init__(self):
        super().__init__('mode_control')
        self.fig = plt.figure()
        
        self.mag_cal_done = 0

        self.analog_axes = [0 , 1, 3, 4]
        self.button_gps_mode = [0 , 3]
        self.button_local_mode = [1 , 2]
        self.button_manual_mode = [1 , 3] # Back button
        self.button_auto_mode = [0, 2] #A or green button
        self.button_stop = 6
        self.button_script_control = 7

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
              logger.info("Line{}: {}".format(count, line.rstrip('\n').strip('')))
              self.lat_array.append(Lines[count].rstrip('\n').split(' ')[0])
              self.long_array.append(Lines[count].rstrip('\n').split(' ')[1])
            count += 1

        global goal_number
        goal_number = 0
        self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])

        # Reference: https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/  
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.kp = 0.8
        # self.mag_declination = math.radians(-14.87) #14.87° W degrees is for Neumayer III Station, Antarctica (https://latitude.to/articles-by-country/aq/antarctica/27247/neumayer-station-iii)
        # self.mag_declination = math.radians(-4.23) # 4.23° W at 0 lat/ long. used in simulation
        # self.imu_heading_offset = math.radians(4.235193576741463 - 90) #At robot's initial spawn position, IMU provides ~ 4.23° heading  which is towards West. Need to account for this offset. In practice, it shouldn't matter as heading will be calculated from magnetometer readings which are wrt real directions.
        self.imu_heading_offset = 0 #Will read this in real time when heading calculation is done
        self.mode = 'auto'
        self.anticipated_dist_if_no_slip = 0

        # self.imu_sub = message_filters.Subscriber(self, Imu, 'imu/data') #Simulation and real-robot
        self.imu_sub = message_filters.Subscriber(self, Imu, 'nav/filtered_imu/data') #Real robot only
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, 'gps/data')
        self.joy_sub = message_filters.Subscriber(self, Joy, 'joy_teleop/joy')
        # self.mag_sub = message_filters.Subscriber(self, MagneticField, 'mag')
        
        # self.velocity_publisher_ = self.create_publisher(Twist, 'husky_velocity_controller/cmd_vel_unstamped', 10)
        self.velocity_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # Twist is a datatype for velocity
        self.move_cmd = Twist()        
        
        # self.ts = message_filters.ApproximateTimeSynchronizer([self.imu_sub, self.gps_sub], 10, 0.09)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joy_sub, self.imu_sub, self.gps_sub], 10, 0.9)
        self.ts.registerCallback(self.listener_callback)
        # time.sleep(2)

    def DMS_to_decimal_format(self, lat, long): 
        '''Check for degrees, minutes, seconds format and convert to decimal'''
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
        # rclpy.logging.get_logger('DMS_to_decimal_format').info('Given GPS goal: lat %s, long %s.' % (lat, long))
        return lat, long

    def rotate(self,target_angle_rad,direction, speed_linear):
        '''Publishes twist command to rotate robot in CW or CCW direction. '''
        global bearing
        if(direction == 'CCW' or direction == 'ccw'):
                multiplier = 1  #Positive velocity means anti-clockwise (CCW) rotation (If we rotate in actual CCW direction, robot moves away from the bearing.. its a workaround)
                #print("CCW")
        else:
                multiplier = -1  #Negative velocity means clockwise (CW) rotation
                #print("CW")
    
        self.move_cmd.angular.z = multiplier * (self.kp * (abs(bearing - self.true_heading) + 0.3))
        # self.move_cmd.angular.z = multiplier * (self.kp * (abs(target_angle_rad) + 0.3))
        # self.move_cmd.linear.x = abs(self.move_cmd.angular.z / 4)
        self.move_cmd.linear.x = speed_linear
        self.velocity_publisher_.publish(self.move_cmd)


    def listener_callback(self,joy_msg, imu_msg, gps_msg):
    # def listener_callback(self, imu_msg, gps_msg):
        self.sync_done = 1
        # rclpy.logging.get_logger('Callback_Function').info('Entered the callback function')
        
        #Extracting data from IMU sensor
        self.imu_msg = imu_msg
        orientation_q = imu_msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list) #Result in radians. -180 < Roll < 180
        global init_heading
        # self.true_heading = -self.roll + init_heading
        self.true_heading = init_heading - (-self.roll - self.imu_heading_offset) #Real robot
        # self.true_heading = init_heading - (self.roll - self.imu_heading_offset) #Simulation
        
        # self.mag_north_heading = -self.roll + self.imu_heading_offset #ASSUMPTION 2) IMU heading is considered as magnetometer heading (as couldn't insert magnetometer is gazebo)
        # logger.warning("IMU data extraction (values in degree): Roll: %f, Pitch: %f, Yaw: %f", math.degrees(self.roll), math.degrees(self.pitch), math.degrees(self.yaw))

        
        #Extracting magnetometer data
        # self.mag_data = mag_msg.magnetic_field

        #Extracting data from Joystick
        self.joy_msg = joy_msg
        # self.enable_script = 1 # Used just for quick testing
        
        #Extracting data from GPS sensor
        self.gps_msg = gps_msg
        self.current_lat = gps_msg.latitude
        self.current_long = gps_msg.longitude
        self.delta_long = self.dest_long - self.current_long
        X = math.cos(self.dest_lat) * math.sin(self.delta_long)
        Y = (math.cos(self.current_lat) * math.sin(self.dest_lat)) - (math.sin(self.current_lat) * math.cos(self.dest_lat) * math.cos(self.delta_long))
        global bearing #Bearing is the required direction towards which robot should orient
        bearing = math.atan2(X, Y)
        # Setting mode control based on input from JS
        if(joy_msg.buttons[self.button_stop] == 1):
            self.mode = 'e_stop'
        elif(joy_msg.buttons[self.button_gps_mode[0]] == 1 and joy_msg.buttons[self.button_gps_mode[1]] == 1 and joy_msg.buttons[self.button_script_control] == 1):
            self.mode = 'gps'
        elif(joy_msg.buttons[self.button_local_mode[0]] == 1 and joy_msg.buttons[self.button_local_mode[1]] == 1 and joy_msg.buttons[self.button_script_control] == 1):
            self.mode = 'local'
        elif(joy_msg.buttons[self.button_auto_mode[0]] == 1 and joy_msg.buttons[self.button_auto_mode[1]] == 1 and joy_msg.buttons[self.button_script_control] == 1):
            self.mode = 'auto'
        elif(joy_msg.buttons[self.button_script_control] == 1 and ((joy_msg.buttons[self.button_manual_mode[0]] == 1 and joy_msg.buttons[self.button_manual_mode[1]] == 1) or joy_msg.axes[self.analog_axes[0]] != 0 or joy_msg.axes[self.analog_axes[1]] != 0 or joy_msg.axes[self.analog_axes[2]] != 0 or joy_msg.axes[self.analog_axes[3]] != 0)):
            self.mode = 'manual'
        else:
            self.mode = 'auto'

    def move_forward(self, distance_to_move, speed_linear): 
        '''If distance_to_move='until_obstacle', then robot will move forward indefinitely until an obstacle is detected, otherwise give distance in meters'''
        if distance_to_move > 3:
            self.move_cmd.linear.x = speed_linear
            self.move_cmd.angular.z = 0.0
            for i in range(0,29): #Used to publish velocity commands 5 times in order to avoid jerky motion
                self.velocity_publisher_.publish(self.move_cmd)
                time.sleep(0.1)
        elif(distance_to_move <= 3): #Distance threshold for the robot to decide whether a destination is reached or not
            global goal_number
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.velocity_publisher_.publish(self.move_cmd)
            if goal_number + 1 < len(self.lat_array):
                logger.warning("Destination #: %d reached :)", goal_number)
                goal_number+=1
            else:
                logger.warning("GPS navigation complete for all goals :)")
            if goal_number < len(self.lat_array):
                self.dest_lat, self.dest_long = self.DMS_to_decimal_format(self.lat_array[goal_number], self.long_array[goal_number])
                global initial_dist_remaining
                initial_dist_remaining = self.calc_goal(self.current_lat, self.current_long, self.dest_lat, self.dest_long)
                global start_time
                start_time = time.time()
                self.anticipated_dist_if_no_slip = initial_dist_remaining - speed_linear*4 #Anticipate remaining distance after 5 seconds of robot's linear motion

    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        '''Calculate distance and azimuth between GPS points. Returns distance in meter'''
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        logger.warning("The distance from robot to goal #: %d is: %f m.", goal_number, distance)
        azimuth = g['azi1']
        # self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))
        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        # self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))
        return distance

    def mag_calibration(self, mag_x_arr, mag_y_arr, lin_acc_x_arr, lin_acc_y_arr, lin_acc_z_arr):
        '''Calculates calibration parameters for magetometer ( tilt correction (bank and elevation angle correction), hard and soft iron correction'''
        
        #Plotting original Magnetometer data
        self.ax = self.fig.add_subplot(221)
        self.ax.set_xlabel('Mag X (Gauss)')
        self.ax.set_ylabel('Mag Y (Gauss)')
        self.ax.set_title('Original Mag data')
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.scatter(mag_x_arr, mag_y_arr) 


        # self.ax2 = self.fig2.add_subplot(221, projection='3d')
        # self.ax2.set_xlabel('X axis')
        # self.ax2.set_ylabel('Y axis')
        # self.ax2.set_zlabel('Z axis')
        # self.ax2.set_title('Original Point Cloud, size: %.i' % data.shape[0])
        # self.ax2.set_xlim(-5,5)
        # self.ax2.set_ylim(-5,5)
        # self.ax2.set_zlim(-1.5,1.2)
        # self.ax2.scatter(data[:,0], data[:,1], data[:,2]) #Plotting original cloud
        
        
        # Reference : https://github.com/rlrosa/uquad/blob/master/doc_externa/Calibracion_sensores/MTD-0801_1_0_Calculating_Heading_Elevation_Bank_Angle.pdf, https://kionixfs.kionix.com/en/document/AN005-Tilt-Sensing-with-Kionix-MEMS-Accelerometers.pdf, https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects


        # Bank and Elevation angle calculation. It's also important to note that tilt/sensitivity errors vary with location, thus it is not possible to employ a static correction factor such as a lookup table. So, not doing as our test environment in not perfectly plane
        # bank_angle = math.atan2(statistics.mean(lin_acc_y_arr), math.sqrt(statistics.mean(lin_acc_x_arr)**2 + statistics.mean(lin_acc_z_arr)**2))
        # elevation_angle = -math.atan2(statistics.mean(lin_acc_x_arr), math.sqrt(statistics.mean(lin_acc_y_arr)**2 + statistics.mean(lin_acc_z_arr)**2))
        
        # Rot_x =  ([1 , 0 , 0], [0, math.cos(bank_angle), math.sin(bank_angle)], [0, -math.sin(bank_angle), math.cos(bank_angle)])
        # Rot_y =  ([math.cos(elevation_angle), 0, -math.sin(elevation_angle)],[0, 1, 0],[math.sin(elevation_angle), 0, math.cos(elevation_angle)])
        # R = np.dot(Rot_x,Rot_y) #This correction needs to be applied on all the data

        # mag_x_tilt_correction = []
        # mag_y_tilt_correction = []
        # mag_z_tilt_correction = []
        # # mag_xyz_tilt_correction = []
        # for i in range(len(mag_x_arr)):
        #     mag_xyz_tilt_correction = np.dot(R, [[mag_x_arr[i]],[mag_y_arr[i]],[mag_y_arr[i]]])
        #     print(mag_xyz_tilt_correction[0][0])
        #     mag_x_tilt_correction.append(mag_xyz_tilt_correction[0][0])
        #     mag_y_tilt_correction.append(mag_xyz_tilt_correction[1][0])
        #     mag_z_tilt_correction.append(mag_xyz_tilt_correction[2][0])
        #     print(np.dot(R, [[mag_x_arr[i]],[mag_y_arr[i]],[mag_y_arr[i]]]))
        #     time.sleep(10)
        

        # ----------------Hard Iron correction-----------------
        magx_hard_offset = (max(mag_x_arr) + min(mag_x_arr))/2
        magy_hard_offset = (max(mag_x_arr) + min(mag_x_arr))/2

        magx_hard_correction = []
        magy_hard_correction = []
        for i in range(len(mag_x_arr)):
            magx_hard_correction.append(mag_x_arr[i] - magx_hard_offset)
            magy_hard_correction.append(mag_y_arr[i] - magy_hard_offset)

        
        logger.warning("Hard iron offsets are: x: %f, y: %f",magx_hard_offset, magy_hard_offset)
        # print("Original Magx: %f", mag_x_arr)
        # print("Mag x hard corrected: ", magx_hard_correction)
        
        # print("Original Magy: ", mag_y_arr)
        # print("Mag y hard corrected: ", magy_hard_correction)


        #Plotting hard iron corrected Magnetometer data
        self.ax = self.fig.add_subplot(222)
        self.ax.set_xlabel('Mag X (Gauss)')
        self.ax.set_ylabel('Mag Y (Gauss)')
        self.ax.set_title('Mag X vs Mag Y after Hard Iron correction')
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.scatter(magx_hard_correction, magy_hard_correction)

        #------------Soft iron correction-----------------%
        r = []
        for i in range(len(mag_x_arr)):
            r.append(math.sqrt(magx_hard_correction[i]**2 + magy_hard_correction[i]**2))
        

        theta = math.asin(magy_hard_correction[r.index(max(r))]/max(r))
        logger.warning("Soft iron correction: theta is: %f radian or %f degree",theta, math.degrees(theta))

        rot_soft = ([math.cos(theta), math.sin(theta)] , [-math.sin(theta), math.cos(theta)]) 

        mag_x_soft_correction = []
        mag_y_soft_correction = []
        for i in range(len(mag_x_arr)):
            mag_xy_soft_correction = np.dot(rot_soft, [[magx_hard_correction[i]],[magy_hard_correction[i]]])
            mag_x_soft_correction.append(mag_xy_soft_correction[0][0])
            mag_y_soft_correction.append(mag_xy_soft_correction[1][0])
        
        # print("Before scaling: Mag x soft corrected: ", mag_x_soft_correction)
        # print("Before scaling: Mag y soft corrected: ", mag_y_soft_correction)

        # ----------Scaling Factor--------------%
        sigma = (max(mag_x_soft_correction)-min(mag_x_soft_correction))/(max(mag_y_soft_correction)-min(mag_y_soft_correction))
        logger.warning("Scaling factor calculation: Num is %f, Den is %f",max(mag_x_soft_correction)-min(mag_x_soft_correction), max(mag_y_soft_correction)-min(mag_y_soft_correction) )
        logger.warning("Soft iron correction: Scaling factor (sigma) is: %f",sigma)

        mag_x_soft_correction[:] = [x / sigma for x in mag_x_soft_correction]
        # print(len(mag_x_soft_correction), mag_x_soft_correction)
        
        rot_soft_back = ([math.cos(-theta), math.sin(-theta)], [-math.sin(-theta), math.cos(-theta)])
        for i in range(len(mag_x_arr)):
            mag_xy_soft_correction = np.dot(rot_soft_back, [[mag_x_soft_correction[i]],[mag_y_soft_correction[i]]])
            mag_x_soft_correction[i] = mag_xy_soft_correction[0][0]
            mag_y_soft_correction[i] = mag_xy_soft_correction[1][0]
        
        # print("Final Mag x soft corrected: ", mag_x_soft_correction)
        # print("Final Mag y soft corrected: ", mag_y_soft_correction)
        self.mag_cal_done = 1
        return magx_hard_offset, magy_hard_offset, theta, sigma #Use these calibration parameters to modify real-time sensor data

    def detect_slip(self, switched_to_rotation, speed_linear):
        '''If slip is detected, robot will move backwards at 1 m/s, otherwise continue moving straight'''
        
        global initial_dist_remaining
        self.move_forward(self.distance_to_move, speed_linear)
        global start_time, time_elapsed
        time_elapsed = time.time() - start_time
        print("Start time: ", start_time, " ; Time elapsed: ", time_elapsed, " ; Switched to rotation: ", switched_to_rotation, "; Initial distance to move: ", initial_dist_remaining, " ; Current dist to move" , self.distance_to_move,  "; Anticipated: ", self.anticipated_dist_if_no_slip , " ; Dist moved in 10 sec: ", abs(initial_dist_remaining - self.distance_to_move), " ; Diff anticipated: ", initial_dist_remaining - self.anticipated_dist_if_no_slip)
        if (time_elapsed >= 10 and switched_to_rotation == 0): # and (self.distance_to_move - self.anticipated_dist_if_no_slip <= 0.4)): #If robot has moved less than 0.4 m in 5 sec, slippage is detected
            # if(abs(initial_dist_remaining - self.distance_to_move) <= initial_dist_remaining - self.anticipated_dist_if_no_slip):
            if(abs(initial_dist_remaining - self.distance_to_move) <= 0.1):
                logger.warning("!!! Slippage detected !!! Robot has either not moved or moved forward by <= 0.1 m within 10 seconds.") 
                logger.warning("Going backwards at 1 m/s for 2 seconds")
                start_time = time.time()
                time_elapsed = 0
                while(time_elapsed <= 2):
                    self.move_forward(self.distance_to_move, -1.0) #Speed should be float
                    time_elapsed = time.time() - start_time 
            start_time = time.time()
            time_elapsed = 0
            initial_dist_remaining = self.distance_to_move
            # print("Initial distance remaining: ", initial_dist_remaining)
            self.anticipated_dist_if_no_slip = initial_dist_remaining - speed_linear*4 #Anticipate remaining distance after 5 seconds of robot's linear motion

def main(args=None):
    
    global init_heading
    init_heading = 0

    # logging.basicConfig(
    #     filename='debug.log',
    #     filemode='w',
    #     # encoding='utf-8',
    #     level=logging.DEBUG,
    #     # level=logging.INFO,
    #     format='%(asctime)s %(levelname)08s %(name)s %(message)s',
    # )

    logger.setLevel(logging.DEBUG) #Levels: https://docs.python.org/3/howto/logging.html : The default level is WARNING, which means that only events of this level and above will be tracked, unless the logging package is configured to do otherwise. To get all logs, set level to DEBUG (lowest level). DEBUG < INFO < WARNING < ERROR < CRITICAL
    
    # Uncomment these lines to get debug logs in the file. Keep them commented to see info on terminal
    path = str(Path(__file__).parent / "./debug.log")
    path = path.replace("install", "src")
    path = path.replace("lib/custom_nav_stack_pkg", "scripts")   
    handler = logging.FileHandler(path, 'w', 'utf-8')
    formatter = logging.Formatter(fmt='%(asctime)s %(levelname)-8s %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    
    logger.warning('############################## LOGGING STARTED ##############################')
    #On terminal, due to some reason, only warning and above are printed, so setting the level to warning. No need to worry.
    rclpy.init(args=args)
    mode_control = ModeControl()
    # rclpy.spin_once(mode_control)
    old_mode = mode_control.mode
    logger.warning('Current mode is: %s', mode_control.mode)

    #Perform Magnetometer calibration to calculate initial heading
    #Step1: Drive robot in a circle for 2-3 times. Record the magnetometer parameters. Calculate the calibration parameters, store them and use for further data from mag sensor

    global i, start_time, time_elapsed
    i = 0
    mag_x_cal_arr = []
    mag_y_cal_arr = []
    mag_z_cal_arr = []
    lin_acc_x_cal_arr = []
    lin_acc_y_cal_arr = []
    lin_acc_z_cal_arr = []
    lat_tmp_src_arr = []
    lon_tmp_src_arr = []
    lat_tmp_dst_arr = []
    lon_tmp_dst_arr = []

    speed_linear = 1.0
    r_mag_cal = 1
    mode_control.sync_done = 0
    # num_loops = 2
    # time_one_loop = 2*math.pi*r_mag_cal/speed_linear
    # print("Time one loop: ", time_one_loop)
    
    # while time_elapsed <= time_one_loop*num_loops:
    # while mode_control.mode == 'auto':
    #     mode_control.sync_done = 0
    #     rclpy.spin_once(mode_control)
        
    #     if mode_control.sync_done == 1:
    #         mag_x_cal_arr.append(mode_control.mag_data.x)
    #         mag_y_cal_arr.append(mode_control.mag_data.y)
    #         mag_z_cal_arr.append(mode_control.mag_data.z)
    #         lin_acc_x_cal_arr.append(mode_control.imu_msg.linear_acceleration.x)
    #         lin_acc_y_cal_arr.append(mode_control.imu_msg.linear_acceleration.y)
    #         lin_acc_z_cal_arr.append(mode_control.imu_msg.linear_acceleration.z)
    #         mode_control.move_cmd.linear.x = speed_linear
    #         mode_control.move_cmd.angular.z = speed_linear/r_mag_cal
    #         logger.warning("Publishing Forward speed: %f, Angular speed: %f", mode_control.move_cmd.linear.x, mode_control.move_cmd.angular.z)
    #         mode_control.velocity_publisher_.publish(mode_control.move_cmd)
    #         logger.warning("Publishing: %s", mode_control.velocity_publisher_.publish(mode_control.move_cmd))
    #         time_elapsed = time.time() - start_time
    #         logger.warning("Time elapsed: %f ", time_elapsed)
        
        # # Code used just for testing. Need to remove in final implementation
        # if time_elapsed >= 2:
        #     mode_control.mode = 'manual'

    # magx_hard_offset, magy_hard_offset, theta, sigma = mode_control.mag_calibration(mag_x_cal_arr, mag_y_cal_arr, lin_acc_x_cal_arr, lin_acc_y_cal_arr, lin_acc_z_cal_arr)
    # logger.warning("Suceessfully recorded magnetometer data. Calculating calibration parameters")
    # magx_hard_offset, magy_hard_offset, theta, sigma = 0.002376, 0.002376, -1.042795, 0.797751
    # logger.warning("Reading hard-coded magnetometer calibration data.")
     
    init_heading_calculated = 0
    logger.warning('Please enable button # %s to run the script.', mode_control.button_script_control)
    
    while 1==1:
        mode_control.sync_done = 0
        rclpy.spin_once(mode_control)
        if mode_control.sync_done == 1:
            
            # mode_control.mode = 'gps' #Used just for quick testing
            if(init_heading_calculated == 0 and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1):
                logger.warning('Starting to find initial heading using GPS co-ordinates. Waiting for 10 sec here and then will move in straight line for 20 sec')
                time_elapsed = 0
                start_time = time.time()
                logger.warning('Mode just before initial gps location calculation is: %s', mode_control.mode)
                while time_elapsed <= 10 and mode_control.mode != 'e_stop':
                    mode_control.sync_done = 0
                    rclpy.spin_once(mode_control)     
                    if mode_control.sync_done == 1:
                        lat_tmp_src_arr.append(mode_control.gps_msg.latitude)
                        lon_tmp_src_arr.append(mode_control.gps_msg.longitude)
                        time_elapsed = time.time() - start_time
                        logger.warning("Tmp initial location time elapsed: %f ", time_elapsed)
                
                logger.warning('Mode just after initial gps location calculation is: %s', mode_control.mode)
                logger.warning("Robot should move forward now for 20 seconds")
                time_elapsed = 0
                start_time = time.time()
                while time_elapsed <= 20 and mode_control.mode != 'e_stop':
                    mode_control.move_forward(10, speed_linear)
                    time_elapsed = time.time() - start_time        
                
                logger.warning("Straight moving time elapsed: %f ", time_elapsed)
                logger.warning('Mode just after straight line moving is: %s', mode_control.mode)
                
                logger.warning('Starting to find initial heading using GPS co-ordinates. Waiting for 10 sec here and then do the calculation')
                time_elapsed = 0
                start_time = time.time()
                while time_elapsed <= 10 and mode_control.mode != 'e_stop':
                    mode_control.sync_done = 0
                    rclpy.spin_once(mode_control)     
                    if mode_control.sync_done == 1:
                        lat_tmp_dst_arr.append(mode_control.gps_msg.latitude)
                        lon_tmp_dst_arr.append(mode_control.gps_msg.longitude)
                        time_elapsed = time.time() - start_time
                logger.warning("Tmp final location time elapsed: %f", time_elapsed)
                logger.warning('Mode just after final gps location calculation is: %s', mode_control.mode)
                
                init_delta_long = statistics.mean(lon_tmp_dst_arr) - statistics.mean(lon_tmp_src_arr)
                X = math.cos(statistics.mean(lat_tmp_dst_arr)) * math.sin(init_delta_long)
                Y = (math.cos(statistics.mean(lat_tmp_src_arr)) * math.sin(statistics.mean(lat_tmp_dst_arr))) - (math.sin(statistics.mean(lat_tmp_src_arr)) * math.cos(statistics.mean(lat_tmp_dst_arr)) * math.cos(init_delta_long))
                
                init_heading = math.atan2(X, Y)
                mode_control.imu_heading_offset = -mode_control.roll #Real robot
                # mode_control.imu_heading_offset = mode_control.roll #Simulation
                init_heading_calculated = 1
                logger.warning("Calculated heading (in degrees) is: %f°", math.degrees(init_heading))
                logger.warning("Please switch modes using Joystick to operate robot further")

            
            if(old_mode != mode_control.mode):
                logger.warning("Changed mode from: %s to %s", old_mode, mode_control.mode)
                switched_to_rotation = 0
            if mode_control.mode == 'e_stop':
                mode_control.move_cmd.linear.x = 0.0
                mode_control.move_cmd.angular.z = 0.0
                mode_control.velocity_publisher_.publish(mode_control.move_cmd)
            elif mode_control.mode == 'manual' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into manual override mode. Complete control in hands of the user
                """ Old code left there to stay which was used when we tried the magnetometer approach to calculate heading which proved unsuccessful.
                Hard iron correction on real-time data
                mode_control.mag_data.x = mode_control.mag_data.x - magx_hard_offset
                mode_control.mag_data.y = mode_control.mag_data.x - magy_hard_offset

                Soft iron correction on real-time data
                rot_soft = ([math.cos(theta), math.sin(theta)] , [-math.sin(theta), math.cos(theta)]) 
                mode_control.mag_data.x = np.dot(rot_soft, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[0][0]
                mode_control.mag_data.y = np.dot(rot_soft, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[1][0]

                Scaling factor correction on real-time data
                mode_control.mag_data.x = mode_control.mag_data.x/sigma

                Rotating back to compensate for rotation during soft correction
                rot_soft_back = ([math.cos(-theta), math.sin(-theta)], [-math.sin(-theta), math.cos(-theta)])
                mode_control.mag_data.x = np.dot(rot_soft_back, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[0][0]
                mode_control.mag_data.y = np.dot(rot_soft_back, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[1][0]

                ---------   1. Estimate the heading (yaw)------------------ 
                Reference: https://digilent.com/blog/how-to-convert-magnetometer-data-into-compass-heading/

                heading_mag = -math.atan2(mode_control.mag_data.y,mode_control.mag_data.x); #Result in -pi to pi
                mode_control.true_heading = heading_mag + mode_control.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
                First try to align robot to desired heading and then move in that direction
                global bearing
                mode_control.true_heading = mode_control.mag_north_heading + mode_control.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
                Calculation to get true heading within -180 to +180 degrees range
                if mode_control.true_heading < math.radians(-180):
                    mode_control.true_heading = math.radians(180 - (abs(math.degrees(mode_control.true_heading)) - 180))
                elif mode_control.true_heading > math.radians(180):
                    mode_control.true_heading = -(math.radians(180) - (mode_control.true_heading - math.radians(180)))
                
                logger.warning("Heading in manual mode (degrees): %f", math.degrees(mode_control.true_heading))
                os.system("ros2 launch husky_control teleop_pub_vel.launch.py &") #No need to run this because both joy nodes are running and they override the speed command from script
                """
                pass
            elif mode_control.mode == 'gps' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into force GPS waypoing mode, can hit obstacles, does not call local mode when detects obstacle. Used for testing purpose   
                """Old code left there to stay which was used when we tried the magnetometer approach to calculate heading which proved unsuccessful.
                Using calculated magnetometer calibration parameters to get correct heading from magnetometer.
                
                logger.warning("Real time: Original magx: %f, magy: %f",mode_control.mag_data.x, mode_control.mag_data.y )
                Hard iron correction on real-time data
                mode_control.mag_data.x = mode_control.mag_data.x - magx_hard_offset
                mode_control.mag_data.y = mode_control.mag_data.x - magy_hard_offset

                Soft iron correction on real-time data
                rot_soft = ([math.cos(theta), math.sin(theta)] , [-math.sin(theta), math.cos(theta)]) 
                mode_control.mag_data.x = np.dot(rot_soft, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[0][0]
                mode_control.mag_data.y = np.dot(rot_soft, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[1][0]

                Scaling factor correction on real-time data
                mode_control.mag_data.x = mode_control.mag_data.x/sigma

                Rotating back to compensate for rotation during soft correction
                rot_soft_back = ([math.cos(-theta), math.sin(-theta)], [-math.sin(-theta), math.cos(-theta)])
                mode_control.mag_data.x = np.dot(rot_soft_back, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[0][0]
                mode_control.mag_data.y = np.dot(rot_soft_back, [[mode_control.mag_data.x],[mode_control.mag_data.y]])[1][0]

                logger.warning("Real time: Corrected magx: %f, magy: %f",mode_control.mag_data.x, mode_control.mag_data.y )

                ---------   1. Estimate the heading (yaw)------------------ 
                Reference: https://digilent.com/blog/how-to-convert-magnetometer-data-into-compass-heading/

                heading_mag = -math.atan2(mode_control.mag_data.y,mode_control.mag_data.x); #Result in -pi to pi
                mode_control.true_heading = heading_mag + mode_control.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North
                First try to align robot to desired heading and then move in that direction
                # mode_control.true_heading = mode_control.mag_north_heading + mode_control.mag_declination #Robot's current heading wrt to True North; If -ve, turn right (cw); + turn left (ccw) to align it with True North"""
                
                global bearing
                
                # Calculation to get true heading within -180 to +180 degrees range
                if mode_control.true_heading < math.radians(-180):
                    mode_control.true_heading = math.radians(180 - (abs(math.degrees(mode_control.true_heading)) - 180))
                elif mode_control.true_heading > math.radians(180):
                    mode_control.true_heading = -(math.radians(180) - (mode_control.true_heading - math.radians(180)))

                # Calculations to find the angle to rotate the robot in order to align with the required heading (true heading)
                if bearing < 0:
                    if (mode_control.true_heading < bearing and mode_control.true_heading > math.radians(-180)) or (mode_control.true_heading > (math.radians(180) - abs(bearing)) and mode_control.true_heading < math.radians(180)):
                        mode_control.direction = 'cw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                        else:
                            mode_control.to_rotate = math.radians(180 - math.degrees(mode_control.true_heading) + 180 - math.degrees(abs(bearing)))
                    elif ((mode_control.true_heading < 0 and mode_control.true_heading > bearing) or (mode_control.true_heading > 0 and mode_control.true_heading < math.radians(180 - math.degrees(abs(bearing))))):
                        mode_control.direction = 'ccw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                        else:
                            mode_control.to_rotate = mode_control.true_heading + abs(bearing)
                elif bearing > 0:
                    if ((mode_control.true_heading < 0 and mode_control.true_heading > -math.radians(180 - math.degrees(bearing))) or (mode_control.true_heading > 0 and mode_control.true_heading < bearing)):
                        mode_control.direction = 'cw' #Original
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = abs(mode_control.true_heading) + bearing
                        else:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)
                    elif ((mode_control.true_heading > bearing and mode_control.true_heading < math.radians(180)) or (mode_control.true_heading < -math.radians(180 - math.degrees(bearing)) and mode_control.true_heading > math.radians(-180))):
                        mode_control.direction = 'ccw' #Original
                        
                        if mode_control.true_heading < 0:
                            mode_control.to_rotate = math.radians(180 - abs(math.degrees(mode_control.true_heading)) + 180 - math.degrees(bearing))
                        else:
                            mode_control.to_rotate = abs(bearing - mode_control.true_heading)

                
                mode_control.distance_to_move = mode_control.calc_goal(mode_control.current_lat, mode_control.current_long, mode_control.dest_lat, mode_control.dest_long)
                
                if mode_control.distance_to_move > 3:
                    if abs(mode_control.to_rotate) > 0.06:
                        switched_to_rotation = 1
                        logger.warning("Info for goal #: %d, Current heading: %f°, Required heading: %f°, Need to rotate by: %f° in Direction: %s", goal_number, math.degrees(mode_control.true_heading), math.degrees(bearing), math.degrees(mode_control.to_rotate), mode_control.direction)
                        mode_control.rotate(mode_control.to_rotate, mode_control.direction, speed_linear)
                    elif mode_control.to_rotate < 0.06:     
                        logger.warning("Robot already aligned towards goal no: %d", goal_number)
                        # print("Switched to rotation: ", switched_to_rotation, "Second condition: ",old_mode != mode_control.mode )
                        if switched_to_rotation == 1 or (old_mode != mode_control.mode):
                            global initial_dist_remaining
                            time_elapsed = 0
                            initial_dist_remaining = mode_control.distance_to_move
                            mode_control.anticipated_dist_if_no_slip = initial_dist_remaining - speed_linear*4 #Anticipate remaining distance after 5 seconds of robot's linear motion
                            # print("Initial distance remaining: ", initial_dist_remaining)
                            start_time = time.time()
                        
                        switched_to_rotation = 0
                        mode_control.detect_slip(switched_to_rotation, speed_linear)   
                elif mode_control.distance_to_move <= 3:
                    mode_control.move_forward(mode_control.distance_to_move, speed_linear)
                    switched_to_rotation = 0

                                       


            elif mode_control.mode == 'local' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into force local navigation mode. Used for testing purpose
                if old_mode != mode_control.mode:
                    print('launching')
                    launch_process = subprocess.Popen(["ros2","launch","/home/administrator/husky_ws2/install/local_nav_pkg/share/local_nav_pkg/launch/local_nav.launch.py"],text=True)

            elif mode_control.mode == 'auto' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter automatic mode which can auto-switch between gps and local mode
                pass
            if old_mode == 'local' and mode_control.mode != 'local':
                launch_process.send_signal(SIGINT)
                launch_process.wait(timeout=30)
        old_mode = mode_control.mode
        # mode_control.destroy_subscription(mode_control.subscription)
            # logger.warning("While end")
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mode_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
