from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
      return LaunchDescription([

           # ****************************************************************** 
           # Microstrain sensor node 
           # ****************************************************************** 

            Node(
                  package    = "microstrain_inertial_examples",
                  executable = "listener.py",
                  namespace  = '',
                  name       = "listener_py",
                  parameters = [
                  ]
           ) #Node End
      ]) #LaunchDescription End
  

 
 
