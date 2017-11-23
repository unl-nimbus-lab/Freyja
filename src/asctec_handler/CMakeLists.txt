cmake_minimum_required(VERSION 2.8.3)
project(asctec_handler)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  lqr_control
  roscpp
  state_manager
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   AsctecCommand.msg
   MotorCommand.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nimbus_serial
  CATKIN_DEPENDS lqr_control roscpp state_manager std_msgs #nimbus_serial
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

set( libnimbusserial_sources src/lib_nimbus_serial.cpp)
## Declare a C++ library
add_library(nimbus_serial STATIC
   ${libnimbusserial_sources}
)


## Declare a C++ executable
add_executable(asctec_handler_node
  src/aj_serial_interface.cpp
  src/asctec_handler_node.cpp
)
target_link_libraries( nimbus_serial
  ${catkin_LIBRARIES}
)
set_target_properties( nimbus_serial 
  PROPERTIES
  COMPILE_FLAGS "-std=c++11 -O3"
)

add_dependencies( asctec_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries( asctec_handler_node
   ${catkin_LIBRARIES}
   nimbus_serial
)

set_target_properties( asctec_handler_node 
  PROPERTIES
  COMPILE_FLAGS "-std=c++11 -O3"
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(asctec_handler ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(asctec_handler_node src/asctec_handler_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(asctec_handler_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(asctec_handler_node
#   ${catkin_LIBRARIES}
# )
