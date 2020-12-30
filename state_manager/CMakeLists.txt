cmake_minimum_required(VERSION 2.8.3)
project(state_manager)

# outdated flag, has no effect. Will be removed in the future.
set( USINGVICON 1 )
set( LQE_ORDER 2 )
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
      -std=c++11 \
      -O3 \
      -DUSE_VICON_=${USINGVICON} \
      -DLQE_INTEGRATOR_ORDER_=${LQE_ORDER}" 
   )

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
  freyja_msgs
  sensor_msgs
  nav_msgs
  std_srvs
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

### Can use state estimator INSTEAD of manager with the right modifications
### This contains an LQE (Kalman) for full state.

#add_executable(bias_estimator_node src/state_estimator.cpp)
#add_dependencies(bias_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
#target_link_libraries(bias_estimator_node
#   ${catkin_LIBRARIES}
# )
