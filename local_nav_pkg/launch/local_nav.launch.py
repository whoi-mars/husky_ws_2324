import launch
import launch_ros
import os


def generate_launch_description(): 

   zed_detect = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='zed_obj_detect.py',
      name='zed_detect'
   )
   
   lidar_detect = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='lidar_detect.py',
      name='lidar_detect'
   )

   get_penguin_positions = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='get_penguin_positions.py',
      name='get_penguin_positions'
   )

   penguin_viz = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='penguin_vis.py',
      name='penguin_viz'
   )
   
   peng_inference = launch_ros.actions.Node(
   	package='local_nav_pkg',
   	executable='inference.py',
   	name='inference'
   )

   collision_avoidance = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='collision_avoid.py',
      name='collision_avoidance'
   )

   approach_speed_controller = launch_ros.actions.Node(
      package='local_nav_pkg',
      executable='approach_speed_controller.py',
      name='approach_speed_controller'
   )
                             
   return launch.LaunchDescription([
        zed_detect, 
        lidar_detect,
        get_penguin_positions,
        penguin_viz,
      #   peng_inference,
        collision_avoidance,
        approach_speed_controller,
    ])

