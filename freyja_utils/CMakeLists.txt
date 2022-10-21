cmake_minimum_required(VERSION 3.8)
project(freyja_utils)
add_compile_options( -std=c++14 -O3 -ffast-math -Wall )

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions( ${EIGEN3_DEFINITIONS} )
include_directories(
  ${Eigen_INCLUDE_DIRS}
)

set( FREYJA_UTILS_SRC
src/filters.cpp 
)

add_library( freyja_utils SHARED
  ${FREYJA_UTILS_SRC}
)


target_include_directories(freyja_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS freyja_utils
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  freyja_utils
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
