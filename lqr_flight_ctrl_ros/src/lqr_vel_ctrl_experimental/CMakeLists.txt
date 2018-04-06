#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.1.0)
project(lqr_vel_ctrl)
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3" )


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  state_manager
  trajectory_provider
  cmake_modules
)
#pkg_search_module( Eigen3 REQUIRED eigen3 )
find_package( Eigen REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN3_DEFINITIONS} )

if( NOT EIGEN3_FOUND )
  # resort to cmake_modules
  find_package( cmake_modules REQUIRED )
  find_package(Eigen REQUIRED)
  set( EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} )
  set( EIGEN3_LIBRARIES ${EIGEN_LIBRARIES} )
else()
  set( EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR} )
endif()

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
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(lqr_vel_ctrl_node src/lqr_control.cpp)
add_dependencies(lqr_control_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lqr_vel_ctrl_node
   ${catkin_LIBRARIES}
 )
