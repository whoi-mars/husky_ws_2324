#!/bin/bash

#source /home/administrator/.profile
#source /home/administrator/.bashrc
sudo modprobe serio 
screen -S virtual_joy -d -m sudo inputattach --fsia6b /dev/flysky_usb_port
screen -S joy_node -d -m /opt/ros/foxy/lib/joy/joy_node --ros-args --remap joy:=joy_remapped
sleep 1
echo 'Flysky red usb dongle should be detected as joystick js1 in /dev/input now'
screen -S joy_remap -d -m python3 /home/administrator/husky_ws2/src/husky/husky_control/joy_remapper.py
