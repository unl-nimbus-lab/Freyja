cmake_minimum_required(VERSION 3.8)
project(lqg_control)
add_compile_options( -std=c++14 -O3 -ffast-math -Wall )


find_package(ament_cmake REQUIRED)
#pkg_search_module( Eigen3 REQUIRED eigen3 )
#find_package( Eigen REQUIRED )


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
#  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

# lqr with bias estimator 
add_executable(lqg_control_node src/lqr_control_bias.cpp src/bias_estimator.cpp)
target_include_directories( lqg_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( lqg_control_node 
  rclcpp
  std_msgs
  freyja_msgs
  geometry_msgs
  std_srvs
)

install(TARGETS
  lqg_control_node
  DESTINATION lib/${PROJECT_NAME})

# lqr with only velocity control 
#add_executable(lqr_vel_ctrl_node src/lqr_vel_control.cpp)
#add_dependencies(lqr_vel_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

ament_package()
