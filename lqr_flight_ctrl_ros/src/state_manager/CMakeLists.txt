cmake_minimum_required(VERSION 2.8.3)
project(state_manager)
set( USINGVICON 1 )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -DUSE_VICON_=${USINGVICON}" )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  common_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES state_manager
  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_manager_node src/state_manager.cpp)
add_dependencies(state_manager_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_manager_node
   ${catkin_LIBRARIES}
 )
