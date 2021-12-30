cmake_minimum_required(VERSION 3.5)
project(freyja_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(freyja_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(temporal_provider src/temporal_provider.cpp)

target_include_directories( temporal_provider PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(temporal_provider
	rclcpp
	freyja_msgs
	std_msgs
	geometry_msgs
)

install(TARGETS
  temporal_provider
  DESTINATION lib/${PROJECT_NAME})

ament_package()
