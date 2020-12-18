cmake_minimum_required(VERSION 2.8.3)
project(freyja_examples)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  freyja_msgs
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_provider
  CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# simple trajectory provider
add_executable(temporal_provider_node src/temporal_provider.cpp)
add_dependencies(temporal_provider_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(temporal_provider_node
   ${catkin_LIBRARIES}
 )