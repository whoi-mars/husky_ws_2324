from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([

           # ****************************************************************** 
           # Microstrain sensor node 
           # ****************************************************************** 

            Node(
                  package    = "microstrain_inertial_examples",
                  executable = "listener_cpp",
                  namespace  = '',
                  name       = "listener_cpp",
                  parameters = [
                  ]
           ) #Node End
      ]) #LaunchDescription End
  

 
 
