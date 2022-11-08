cmake_minimum_required(VERSION 3.5)
project(pipeline_inspection)

find_package(ament_cmake REQUIRED COMPONENTS)
find_package(ament_cmake_python REQUIRED COMPONENTS)
find_package(rclpy REQUIRED)
find_package(mavros_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPath.srv"
  DEPENDENCIES geometry_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/lawn_mower.py
  scripts/pipeline_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
