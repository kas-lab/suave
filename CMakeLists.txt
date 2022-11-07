cmake_minimum_required(VERSION 3.5)
project(pipeline_inspection)

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(ament_cmake_python REQUIRED COMPONENTS)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/spiral_pattern_position.py
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
