cmake_minimum_required(VERSION 3.8)
project(waypoint_manager)
add_compile_options( -std=c++14 -O3 -ffast-math -Wall )


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
add_executable(waypoint_manager_node src/waypoint_manager.cpp)
target_include_directories( waypoint_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies( waypoint_manager_node 
  rclcpp
  std_msgs
  freyja_msgs
  geometry_msgs
  std_srvs
)
install(TARGETS
  waypoint_manager_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()