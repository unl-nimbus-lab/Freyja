cmake_minimum_required(VERSION 2.8.3)
project(freyja_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Generate messages in the 'msg' folder
add_message_files( #DIRECTORY msg
  FILES
  asctec_handler/AsctecCommand.msg
  asctec_handler/AsctecData.msg
  asctec_handler/MotorCommand.msg
  trajectory_provider/ReferenceState.msg
  trajectory_provider/TrajectoryDebug.msg
  trajectory_provider/WaypointTarget.msg
  lqr_ctrl/CtrlCommand.msg
  lqr_ctrl/ControllerDebug.msg
  state_manager/CurrentState.msg
  state_manager/CurrentStateBiasEst.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES freyja_msgs
#  CATKIN_DEPENDS geometry_msgs message_generation message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

