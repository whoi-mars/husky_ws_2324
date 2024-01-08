cmake_minimum_required(VERSION 3.5)
project(local_nav_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Penguin.msg"
  "msg/PenguinList.msg"
  "msg/PenguinInference.msg"
  "msg/PenguinInferenceList.msg"
  "msg/PenguinApproachStatus.msg"
  "msg/Corner.msg"
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
include_directories(include)

install(
  DIRECTORY launch rviz world scripts msg
  DESTINATION share/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/lidar_detect.py
  scripts/penguin_vis.py
  scripts/get_penguin_positions.py
  scripts/inference.py
  scripts/collision_avoid.py
  scripts/approach_speed_controller.py
  scripts/zed_obj_detect.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
