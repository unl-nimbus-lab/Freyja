cmake_minimum_required(VERSION 2.8.3)
project(apm_handler)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

SET(NO_PIXHAWK "false" CACHE BOOL "Building Pixhawk interface handler or not?")

if( NO_PIXHAWK )
  message( AUTHOR_WARNING "\n\t\tFREYJA: Ignoring mavros interface to Pixhawk!\n" )
  message( NOTICE "\tFREYJA: User-requested ignore apm_handler package. Please ensure alternative means of communications!\n" )
else()

find_package(catkin REQUIRED COMPONENTS
  freyja_msgs
  geometry_msgs
  roscpp
  std_msgs
  tf
  mavros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES apm_handler
#  CATKIN_DEPENDS freyja_msgs geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable

add_executable(${PROJECT_NAME}_node src/mavros_translate.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

endif() # NO_PIXHAWK
