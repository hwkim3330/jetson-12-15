cmake_minimum_required(VERSION 3.5)
project(rssaem_web)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  www
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
