from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swri_transform_util',
            executable='initialize_origin.py',
            name='initialize_origin',
            output='screen',
            namespace='',
            parameters=[{'local_xy_frame':'/odom'}, 
                        {'local_xy_origin':'auto'},
                        {'use_sim_time':True}], 
            remappings=[('fix', 'gps/data')]
        ),
    ])
