cmake_minimum_required(VERSION 3.8)
project(freyja_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/asctec_handler/AsctecCommand.msg"
  "msg/asctec_handler/AsctecData.msg"
  "msg/asctec_handler/MotorCommand.msg"
  "msg/trajectory_provider/ReferenceState.msg"
  "msg/trajectory_provider/TrajectoryDebug.msg"
  "msg/trajectory_provider/WaypointTarget.msg"
  "msg/lqr_ctrl/CtrlCommand.msg"
  "msg/lqr_ctrl/ControllerDebug.msg"
  "msg/state_manager/CurrentState.msg"
  "msg/state_manager/CurrentStateBiasEst.msg"
 )
 
ament_package()

