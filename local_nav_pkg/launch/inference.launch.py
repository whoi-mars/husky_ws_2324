import launch
import launch_ros
import os


def generate_launch_description(): 
   
   peng_inference = launch_ros.actions.Node(
   	package='local_nav_pkg',
   	executable='inference.py',
   	name='inference'
   )
                             
   return launch.LaunchDescription([
        peng_inference
    ])

