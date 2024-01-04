#! /usr/bin/env python3

__author__ = "Dev Vaibhav"
__credits__ = ["Dev Vaibhav"]
__maintainer__ = "Dev Vaibhav"
__email__ = "dev.vaibhav@whoi.edu"

import logging
from statistics import mode
import rclpy
from rclpy.node import Node
import subprocess
from signal import SIGINT

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

        self.analog_axes = [0, 1, 3, 4]
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
        
        count = 0
        self.mode = 'auto'
        

        self.joy_sub = message_filters.Subscriber(self, Joy, 'joy_teleop/joy')
        
        self.velocity_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.move_cmd = Twist()        
        
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joy_sub], 10, 0.9)
        self.ts.registerCallback(self.listener_callback)


    def listener_callback(self, joy_msg):
    
        self.sync_done = 1
        # rclpy.logging.get_logger('Callback_Function').info('Entered the callback function')

        #Extracting data from Joystick
        self.joy_msg = joy_msg
        # self.enable_script = 1 # Used just for quick testing
        # print(joy_msg.buttons)
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

    # def local_nav_launch(self):
    #     launch_service = launch.LaunchService()
    #     launch_description = launch.LaunchDescription([
    #         launch.actions.IncludeLaunchDescription(
    #             launch.launch_description_sources.AnyLaunchDescriptionSource(
    #                 launch_file_path
    #             ),
    #             launch_arguments=parsed_launch_arguments,
    #         )
    #     ])
    #     launch_service.include_launch_description(launch_description)
def main(args=None):

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

    mode_control.sync_done = 0

    logger.warning('Please enable button # %s to run the script.', mode_control.button_script_control)
    while 1==1:
        mode_control.sync_done = 0
        
        rclpy.spin_once(mode_control)
        if mode_control.sync_done == 1:
            
            
            if(old_mode != mode_control.mode):
                print(mode_control.mode)
                logger.warning("Changed mode from: %s to %s", old_mode, mode_control.mode)
            if mode_control.mode == 'e_stop':
                mode_control.move_cmd.linear.x = 0.0
                mode_control.move_cmd.angular.z = 0.0
                mode_control.velocity_publisher_.publish(mode_control.move_cmd)
            elif mode_control.mode == 'manual' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into manual override mode. Complete control in hands of the user
                
                pass
            elif mode_control.mode == 'gps' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into force GPS waypoing mode, can hit obstacles, does not call local mode when detects obstacle. Used for testing purpose   
               pass
            elif mode_control.mode == 'local' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter into force local navigation mode. Used for testing purpose
                if old_mode != mode_control.mode:
                    print('launching')
                    launch_process = subprocess.Popen(["ros2","launch","/home/administrator/husky_ws2/install/local_nav_pkg/share/local_nav_pkg/launch/local_nav.launch.py"],text=True)
                    # launch_process.run_async()
            elif mode_control.mode == 'auto' and mode_control.joy_msg.buttons[mode_control.button_script_control] == 1: #Enter automatic mode which can auto-switch between gps and local mode
                pass
            if old_mode == 'local' and mode_control.mode != 'local':
                launch_process.send_signal(SIGINT)
                launch_process.wait(timeout=30)
        old_mode = mode_control.mode
        
    mode_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
