cmake_minimum_required(VERSION 3.8)
project(lqg_control)
add_compile_options( -O3 -ffast-math -Wall )
set( CMAKE_CXX_STANDARD 17 )

set( ignoreMe "${NO_PIXHAWK}${NO_ROBOMASTER}" )

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN3_DEFINITIONS} )

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${Eigen_INCLUDE_DIRS}
)

# lqr with bias estimator: FLIGHT
add_executable(lqg_flight_node src/lqr_ctrl_flight.cpp src/bias_estimator.cpp)
target_include_directories( lqg_flight_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( lqg_flight_node 
  rclcpp
  std_msgs
  freyja_msgs
  geometry_msgs
  std_srvs
)
install(TARGETS
  lqg_flight_node
  DESTINATION lib/${PROJECT_NAME})


# lqr with bias estimator: OMNIWHEEL  
add_executable(lqg_4wheel_node src/lqr_ctrl_omni4wheel.cpp src/bias_estimator.cpp)
target_include_directories( lqg_flight_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( lqg_4wheel_node 
  rclcpp
  std_msgs
  freyja_msgs
  geometry_msgs
  std_srvs
)
install(TARGETS
  lqg_4wheel_node
  DESTINATION lib/${PROJECT_NAME})

# lqr with only velocity control 
#add_executable(lqr_vel_ctrl_node src/lqr_vel_control.cpp)
#add_dependencies(lqr_vel_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

ament_package()
