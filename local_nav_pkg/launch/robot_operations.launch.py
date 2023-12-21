import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share_descrip = launch_ros.substitutions.FindPackageShare(package='husky_description').find('husky_description')
    pkg_share_local_nav = launch_ros.substitutions.FindPackageShare(package='local_nav_pkg').find('local_nav_pkg')
    #bringup_dir = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    #launch_dir = os.path.join(bringup_dir, 'launch')
    #bringup_dir = get_package_share_directory('nav2_bringup')
    
    default_model_path = os.path.join(pkg_share_descrip, 'urdf/husky.urdf.xacro')
    world_path=os.path.join(pkg_share_local_nav, 'world/one_cone.world')
    default_rviz_config_path = os.path.join(pkg_share_local_nav, 'rviz/config.rviz')
    
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    
    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "control.yaml"])
        
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
            "is_sim:=true",
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    
    #declare_map_yaml_cmd = DeclareLaunchArgument(
    #    'map',
    #    default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
    #    description='Full path to map file to load')
    
    spawn_husky_velocity_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description,{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
   
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
    
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen')
   
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time':  LaunchConfiguration('use_sim_time')}]
    )
       
    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_husky_velocity_controller],
        )
    )
    
    
    launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("local_nav_pkg"), 'launch', 'simulation.launch.py'])))
    
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_node',
       output='screen',
       parameters=[os.path.join(pkg_share_local_nav, 'config/ekf.yaml'),{'use_sim_time': True}]
    )
    
    global_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_node',
       output='screen',
       parameters=[os.path.join(pkg_share_local_nav, 'config/ekf_global.yaml'),{'use_sim_time': True}]
    )
    
    robot_navsat_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='navsat_transform_node',
       name='navsat_transform_node',
       respawn='true',
       remappings=[('imu','imu/data'),('gps/fix', 'gps/data')],
       parameters=[{'magnetic_declination_radians': 0.0},{'yaw_offset': 1.5708},{'broadcast_utm_transform': True},{'publish_filtered_gps': True}]
    )

    
    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))
        
    launch_scan_from_velodye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("pointcloud_to_laserscan"), 'launch', 'sample_pointcloud_to_laserscan_launch.py'])))  
        
    launch_slam = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(PathJoinSubstitution(
    	[FindPackageShare("slam_toolbox"), 'launch', 'online_async_launch.py'])))
    	
    launch_navigation = IncludeLaunchDescription(
    	PythonLaunchDescriptionSource(PathJoinSubstitution(
    	[FindPackageShare("nav2_bringup"), 'launch', 'navigation_launch.py'])))
    	
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare("nav2_bringup"), 'launch', 'bringup_launch.py'])),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart
                          }.items()
                          )
                          
    launch_speed_limit = launch_ros.actions.Node(
       package='local_nav_pkg',
       executable='set_speed_limit.py',
       name='set_speed_limit')
       
    enforce_speed_limit = launch_ros.actions.Node(
       package='local_nav_pkg',
       executable='enforce_speed_limit.py',
       name='enforce_speed_limit')
                             
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path, description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='namespace', default_value=''),
        launch.actions.DeclareLaunchArgument(name='use_namespace', default_value='false'),
        launch.actions.DeclareLaunchArgument(name='slam', default_value='false'),
        #launch.actions.DeclareLaunchArgument(name='map', default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml')),
        #launch.actions.DeclareLaunchArgument(name='params_file', default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')),
        launch.actions.DeclareLaunchArgument(name='default_bt_xml_filename', default_value=os.path.join(launch_ros.substitutions.FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator'), 'behavior_trees', 'follow_point.xml')),
        launch.actions.DeclareLaunchArgument(name='autostart', default_value='true'),
        
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_joint_state_broadcaster,
        diffdrive_controller_spawn_callback,
        launch_simulation,
        #gzserver,
        #gzclient,
        #spawn_entity,
        robot_navsat_node,
        robot_localization_node,
        #global_localization_node,
        rviz_node,
        #launch_husky_control,
        launch_husky_teleop_base,
        launch_scan_from_velodye,
        #launch_slam,
        #launch_navigation   
        #bringup_cmd,
        launch_speed_limit,
        enforce_speed_limit     
    ])

