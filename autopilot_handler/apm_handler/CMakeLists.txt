cmake_minimum_required(VERSION 3.8)
project(apm_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17 -O3)

SET(NO_PIXHAWK "false" CACHE BOOL "Building Pixhawk interface handler or not?")

if( NO_PIXHAWK )
  message( AUTHOR_WARNING "\n\t\tFREYJA:: Skip package: ignoring package with mavros dependency!\n" )
  message( NOTICE "\tFREYJA: User-requested ignore apm_handler package. Please ensure alternative means of communications!\n" )
else()

  find_package(ament_cmake REQUIRED)
  find_package( rclcpp REQUIRED )
  find_package( std_msgs REQUIRED )
  find_package( geometry_msgs REQUIRED )
  find_package( freyja_msgs REQUIRED )
  find_package( sensor_msgs REQUIRED )
  find_package( nav_msgs REQUIRED ) 
  find_package( std_srvs REQUIRED ) 
  find_package( tf2 REQUIRED )
  find_package( tf2_ros REQUIRED )
  find_package( tf2_geometry_msgs REQUIRED )
  find_package( mavros_msgs REQUIRED )
  find_package( Eigen3 REQUIRED )
  include_directories( include )

  ## Declare a C++ executable
  add_executable(${PROJECT_NAME}_node src/mavros_translate.cpp)
  target_include_directories( ${PROJECT_NAME}_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies( ${PROJECT_NAME}_node 
    rclcpp
    std_msgs
    freyja_msgs
    geometry_msgs
    std_srvs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    mavros_msgs
  )

  install(TARGETS
    apm_handler_node
    DESTINATION lib/apm_handler
  )

ament_package()
endif() # NO_PIXHAWK
