from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_goal',
            executable='gps_goal_calculation.py',
            name='gps_goal',
            output='screen',
            namespace='',
            parameters=[{'frame_id':'map'}, #The tf frame for move_base goals. 
                        {'use_sim_time':True}], 
            remappings=[('gps_goal_pose', 'gps_goal_pose'), #Set a GPS goal using a PoseStamped message. The x and y values will be considered latitude and longitude respectively. Z, roll, pitch, and yaw values will be respected. 
                        ('gps_goal_fix', 'gps_goal_fix'), #Set a GPS goal using a NavSatFix GPS message. The goal for roll, pitch, and yaw will be 0 degrees because they are not in this message type. 
                        ('local_xy_origin', 'local_xy_origin'),] #The topic to get the origin GPS location from. This location is the origin (0,0) of the frame (typically world) given by the local_xy_frame parameter to the initialize_origin node. This location is used to calculate distances for goals. One message on this topic is consumed when the node starts only. 
        ),
    ])
