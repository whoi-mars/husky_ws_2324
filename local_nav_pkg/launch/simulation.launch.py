import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share_local_nav = launch_ros.substitutions.FindPackageShare(package='local_nav_pkg').find('local_nav_pkg')
    world_path=os.path.join(pkg_share_local_nav, 'world/three_cone.world')
    
    
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    
    spawn_entity = launch_ros.actions.Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'husky', '-topic', 'robot_description'],
      output='screen',
      parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
                             
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        
        gzserver,
        gzclient,
        spawn_entity,
             
    ])

