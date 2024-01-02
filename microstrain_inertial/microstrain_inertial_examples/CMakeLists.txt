cmake_minimum_required(VERSION 3.5)
project(microstrain_inertial_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(microstrain_inertial_msgs REQUIRED)

ament_export_include_directories()
ament_export_dependencies(
    roscpp
    rospy
    sensor_msgs
    microstrain_inertial_msgs
    cmake_modules
)

# CPP Listener
set(LISTENER_CPP listener_cpp)
set(LISTENER_CPP_SRC_FILES src/listener.cpp)
add_executable(${LISTENER_CPP} ${LISTENER_CPP_SRC_FILES})
ament_target_dependencies(${LISTENER_CPP}
  rclcpp
  sensor_msgs
  microstrain_inertial_msgs
)

install(
  TARGETS ${LISTENER_CPP}
  DESTINATION lib/${PROJECT_NAME}
)

# Install python executables
install(PROGRAMS
  scripts/listener.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()