cmake_minimum_required(VERSION 3.5)
project(freyja_examples)
add_compile_options( -std=c++14 -O3 -ffast-math -Wall )

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(freyja_msgs REQUIRED)

include_directories(include)

add_executable(temporal_provider src/temporal_provider.cpp)

target_include_directories( temporal_provider PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies( temporal_provider
	rclcpp
	freyja_msgs
	std_msgs
)

install(TARGETS
  temporal_provider
  DESTINATION lib/${PROJECT_NAME})

ament_package()
