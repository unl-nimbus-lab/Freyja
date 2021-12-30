cmake_minimum_required(VERSION 3.8)
project(freyja_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

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
  "msg/lqr_ctrl/WheelCommand.msg"
  "msg/state_manager/CurrentState.msg"
  "msg/state_manager/CurrentStateBiasEst.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
 )

#  install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

ament_export_dependencies(rosidl_default_runtime)

ament_package()

