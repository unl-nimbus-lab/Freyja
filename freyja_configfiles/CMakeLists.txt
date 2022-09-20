cmake_minimum_required(VERSION 3.8)
project(freyja_configfiles)

set( ignoreMe "${NO_PIXHAWK}${NO_ROBOMASTER}" )

find_package( ament_cmake REQUIRED )

install( DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
