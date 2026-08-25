cmake_minimum_required(VERSION 3.8)
project(create2_demo)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(gazebo_fps_limiter SHARED
  plugins/gazebo_fps_limiter.cpp
)
target_include_directories(gazebo_fps_limiter PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(gazebo_fps_limiter ${GAZEBO_LIBRARIES} Qt5::Widgets)
target_compile_options(gazebo_fps_limiter PRIVATE ${GAZEBO_CXX_FLAGS})

install(TARGETS gazebo_fps_limiter
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/activate_navigation.py
  scripts/clockwise_wall_tracer.py
  scripts/contact_monitor.py
  scripts/exploration_coordinator.py
  scripts/scan_clearer.py
  scripts/trial_monitor.py
  scripts/wandering_mapper.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch maps urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
