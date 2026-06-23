cmake_minimum_required(VERSION 3.5)
project(create_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
    PATTERN "*.launch"
    PATTERN "*.launch.py"
  PATTERN ".svn" EXCLUDE
  PATTERN "build" EXCLUDE
  PATTERN "install" EXCLUDE
  PATTERN "log" EXCLUDE
)

ament_package()
