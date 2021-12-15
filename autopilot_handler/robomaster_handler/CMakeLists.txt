cmake_minimum_required(VERSION 3.8)
project(robomaster_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package(robomaster_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)
 

# handler node
add_executable( s1_interface src/s1_handler.cpp )

ament_target_dependencies( s1_interface 
  rclcpp
  std_msgs
  freyja_msgs
  robomaster_msgs
)

install(TARGETS
  s1_interface
  DESTINATION lib/${PROJECT_NAME})


ament_package()

