cmake_minimum_required(VERSION 3.8)
project(robomaster_handler)

set( ignoreMe "${NO_PIXHAWK}" )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

SET(NO_ROBOMASTER "false" CACHE BOOL "Building robomaster interface handler or not?")

if( NO_ROBOMASTER )
  message( AUTHOR_WARNING "\n\tFREYJA: Ignoring robomaster interfaces!\n" )
  message( NOTICE "\tFREYJA: User-requested ignore robomaster interfaces. Please ensure there's an alternative handler for ground vehicles!\n" )
else()
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

endif() # NO_ROBOMASTER
