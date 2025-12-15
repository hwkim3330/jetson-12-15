cmake_minimum_required(VERSION 3.8)
project(rssaem_ai)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/person_detector.py
  scripts/person_follower.py
  scripts/body_tracker.py
  scripts/jetson_camera.py
  scripts/yolo_detector.py
  scripts/gesture_detector.py
  scripts/setup_models.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
