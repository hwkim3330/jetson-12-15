cmake_minimum_required(VERSION 3.5)
project(rssaem_web)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/slam_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  www
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
