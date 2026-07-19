cmake_minimum_required(VERSION 3.8)
project(create2_demo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/activate_navigation.py
  scripts/clockwise_wall_tracer.py
  scripts/scan_clearer.py
  scripts/trial_monitor.py
  scripts/wandering_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
