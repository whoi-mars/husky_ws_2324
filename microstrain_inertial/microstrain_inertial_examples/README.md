# ROS-MSCL Examples Readme

A example listener node is provided in to demonstrate a very basic C++ node that subscribes to and displays some of the data published by the ros_mscl driver.

Over time we will provide more robust and varied examples in both C++ and Python, but in the meantime we hope this helps in quickly testing to ensure everything is installed and working properly!

Prerequisite: completed setup and build steps found [here](../).

#### Create the example package
1. If the entire ROS-MSCL package, including the `microstrain_inertial_examples` directory, is not already there move the `microstrain_inertial_examples` package to the `your_workspace/src` folder.

2. Locate and register the package to the workspace: `rospack find microstrain_inertial_examples`

3. Build your workspace:
        
        cd ~/your_workspace
        colcon build
        source ~/your_workspace/install/setup.bash
   The source command may need to be run in each terminal prior to launching a ROS node.
   You may need to change the permissions on the listener.py file if it is failing to run.


#### Launch the listener node
Launch the inertial device node:
            
    ros2 launch microstrain_inertial_driver microstrain_launch.py

In a separate terminal, launch the example listener node:

    ros2 launch microstrain_inertial_examples listener_cpp_launch.py

In another seperate terminal, configure and activate the device node

    ros2 lifecycle set /gx5/microstrain_inertial_driver_node configure
    ros2 lifecycle set /gx5/microstrain_inertial_driver_node activate

