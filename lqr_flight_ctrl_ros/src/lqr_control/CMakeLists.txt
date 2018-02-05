cmake_minimum_required(VERSION 2.8.3)
project(lqr_control)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  state_manager
  trajectory_provider
)

find_package(Eigen3 REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    CtrlCommand.msg
    ControllerDebug.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES lqr_control
  CATKIN_DEPENDS Eigen3 message_generation roscpp state_manager
  DEPENDS Eigen3
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(lqr_control_node src/lqr_control.cpp)
add_dependencies(lqr_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lqr_control_node
   ${catkin_LIBRARIES}
 )
