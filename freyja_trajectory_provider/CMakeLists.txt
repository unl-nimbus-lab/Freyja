cmake_minimum_required(VERSION 2.8.3)
project(freyja_trajectory_provider)
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
add_executable(trajectory_provider_node src/trajectory_provider.cpp)
add_dependencies(trajectory_provider_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(trajectory_provider_node
   ${catkin_LIBRARIES}
 )


# vision-based guidance for approach
# add_executable(trajectory_shaper_node src/trajectory_shaper.cpp)
# add_dependencies(trajectory_shaper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(trajectory_shaper_node
#   ${catkin_LIBRARIES}
# )
