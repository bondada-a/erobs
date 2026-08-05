cmake_minimum_required(VERSION 3.8)
project(erob_epics)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/action_status_to_pv.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
