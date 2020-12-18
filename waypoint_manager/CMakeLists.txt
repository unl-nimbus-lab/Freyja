cmake_minimum_required(VERSION 3.0.2)
project(waypoint_manager)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  freyja_msgs
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  cmake_modules
)

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


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES idsc_path_gen
#  CATKIN_DEPENDS common_msgs geometry_msgs message_generation roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/waypoint_manager.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
