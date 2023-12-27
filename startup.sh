#!/bin/bash
source /home/administrator/.profile
source /home/administrator/.bashrc
export DISPLAY=:0

sleep 5
sudo modprobe serio 
sleep 1
screen -S virtual_joy -d -m sudo inputattach --fsia6b /dev/flysky_usb_port
sleep 5
echo 'Flysky red usb dongle should be detected as joystick js1 in /dev/input now'

sudo rm /dev/input/flysky_red_js_port
sudo ln -s /dev/input/$(cat /proc/bus/input/devices | sed -n -e '/N: Name="FS-iA6B iBus RC receiver"/,$p' | grep 'H:' | head -1 | grep -oh "js[0-9]") /dev/input/flysky_red_js_port
echo 'Symlink for js* created as flysky_red_js_port'

sleep 1
screen -S base -d -m ros2 launch husky_base base.launch.py
echo 'husky_base is ran. Light should turn GREEN now for COMM'
echo 'Should be able to operate the robot using Flysky controller now'

sleep 1
screen -S ricoh -d -m ros2 run penguin_detection img_publisher
echo 'Ricoh theta should spit data now'

sleep 1
screen -S ricoh -d -m ros2 run theta_driver theta_driver_node

sleep 1
#screen -S zed -d -m ros2 launch zed_wrapper zed2.launch.py
echo 'ZED should spit data now'

sleep 1
screen -S imu -d -m ros2 launch microstrain_inertial_driver microstrain_launch.py
echo 'IMU launch file ran'

sleep 1
ros2 lifecycle set /microstrain_inertial_driver configure
echo 'IMU lifecycle node set to configure'

sleep 1
ros2 lifecycle set /microstrain_inertial_driver activate
echo 'IMU lifecycle node set to activate. It should spit data now'

sleep 1
screen -S velodyne -d -m ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
echo 'Velodyne should spit data now'

sleep 1
screen -S gps -d -m /home/administrator/husky_ws2/src/gps_driver.py
echo 'GPS should spit data now'

sleep 1
screen -S rviz -d -m ros2 run rviz2 rviz2 -d /home/administrator/husky_ws2/src/custom_nav_stack_pkg/rviz/husky_real_robot.rviz
echo 'RVIZ should open now and display sensor data'

sleep 1
screen -S mode_control -m ros2 run custom_nav_stack_pkg mode_control.py
echo 'Mode control script is running now'

#screen -S husky_control_joystick -d -m /home/administrator/husky_ws2/src/husky_control_joystick.sh
sleep 10000

#screen -S joy_node -d -m /opt/ros/foxy/lib/joy/joy_node --ros-args --remap joy:=joy_remapped


#screen -S joy_remap -d -m python3 /home/administrator/husky_ws2/src/husky/husky_control/joy_remapper.py







