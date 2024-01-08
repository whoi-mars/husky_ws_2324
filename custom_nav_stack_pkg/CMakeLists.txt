cmake_minimum_required(VERSION 3.5)
project(custom_nav_stack_pkg)

# # Default to C99
# if(NOT CMAKE_C_STANDARD)
#   set(CMAKE_C_STANDARD 99)
# endif()

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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED) #To include markers visualization in rviz
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# set(Boost_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
# set(Boost_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
# set(BOOST_LIBRARY_DIRS /usr/lib/x86_64-linux-gnu/)
# set(BOOST_LIBRARY_DIR /usr/lib/x86_64-linux-gnu/)
# set(Boost_INCLUDE_DIR /usr/include/)
# set(BOOST_INCLUDE_DIR /usr/include/)
# set(Boost_INCLUDE_DIRS /usr/include/)
# set(BOOST_INCLUDE_DIRS /usr/include/)
# find_package(Boost 1.55.0 ${QUIET_} COMPONENTS system filesystem date_time iostreams)
# find_package(Boost 1.55.0 REQUIRED COMPONENTS system filesystem date_time iostreams) #Surprisingly this can find Boost but one present in PCL can't


# message( CMAKE_MODULE_PATH in mycmake:  ${CMAKE_MODULE_PATH} )
# message( Boost_FOUND in my cmakelist:  ${Boost_FOUND} )
find_package(PCL REQUIRED)

# https://pointclouds.org/documentation/tutorials/using_pcl_pcl_config.html
# When PCL is found, several related variables are set:

#     PCL_FOUND: set to 1 if PCL is found, otherwise unset

#     PCL_INCLUDE_DIRS: set to the paths to PCL installed headers and the dependency headers

#     PCL_LIBRARIES: set to the file names of the built and installed PCL libraries

#     PCL_LIBRARY_DIRS: set to the paths to where PCL libraries and 3rd party dependencies reside

#     PCL_VERSION: the version of the found PCL

#     PCL_COMPONENTS: lists all available components

#     PCL_DEFINITIONS: lists the needed preprocessor definitions and compiler flags



set(dependencies "rclcpp" "sensor_msgs" "geometry_msgs" "nav_msgs" "nav2_msgs" "lifecycle_msgs" "visualization_msgs" "PCL" )#"libpcl-dev"


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(include ${colcon_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
# /usr/include/pcl-1.10 /usr/include/eigen3 
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
add_executable(cpp_executable2 src/cpp_node2.cpp)
#ament_target_dependencies(cpp_executable rclcpp)
ament_target_dependencies(cpp_executable ${dependencies})
ament_target_dependencies(cpp_executable2 ${dependencies})

# Install Cpp executables
install(TARGETS
  cpp_executable
  cpp_executable2
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries (cpp_executable ${PCL_LIBRARIES})
target_link_libraries (cpp_executable2 ${PCL_LIBRARIES})

# Install Python executables
install(PROGRAMS
  scripts/custom_nav_stack.py
  scripts/mode_control.py
  scripts/mode_control_gps_removed.py
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
