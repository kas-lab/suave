cmake_minimum_required(VERSION 3.8)
project(suave_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
# list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(system_modes_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp
  mavros_msgs
  rclcpp
  std_msgs
  std_srvs
  system_modes_msgs
  diagnostic_msgs
)

include_directories(
  include
)

add_library(suave_mission SHARED src/suave_bt/suave_mission.cpp)

add_library(is_pipeline_found_bt_node SHARED src/suave_bt/is_pipeline_found.cpp)
add_library(is_pipeline_inspected_bt_node SHARED src/suave_bt/is_pipeline_inspected.cpp)
add_library(condition_water_visibility_bt_node SHARED src/suave_bt/condition_water_visibility.cpp)
add_library(condition_thrusters_ok_bt_node SHARED src/suave_bt/condition_thrusters_ok.cpp)

add_library(action_arm_thrusters_bt_node SHARED src/suave_bt/action_arm_thrusters.cpp)
add_library(action_set_guided_mode_bt_node SHARED src/suave_bt/action_set_guided_mode.cpp)
add_library(action_search_pipeline_bt_node SHARED src/suave_bt/action_search_pipeline.cpp)
add_library(action_inspect_pipeline_bt_node SHARED src/suave_bt/action_inspect_pipeline.cpp)
add_library(action_recharge_battery_bt_node SHARED src/suave_bt/action_recharge_battery.cpp)
add_library(action_change_mode_bt_node SHARED src/suave_bt/action_change_mode.cpp)


list(APPEND plugin_libs
  suave_mission
  is_pipeline_found_bt_node
  is_pipeline_inspected_bt_node
  condition_water_visibility_bt_node
  condition_thrusters_ok_bt_node
  action_arm_thrusters_bt_node
  action_set_guided_mode_bt_node
  action_search_pipeline_bt_node
  action_inspect_pipeline_bt_node
  action_recharge_battery_bt_node
  action_change_mode_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(suave_bt src/suave_bt.cpp)
ament_target_dependencies(suave_bt ${dependencies})
target_link_libraries(suave_bt ${plugin_libs})


install(TARGETS
  suave_bt
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# add config, launch etc, whatever is needed
install(DIRECTORY
  bts
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_libraries(
  ${plugin_libs}
)
ament_export_targets(
  export_${PROJECT_NAME}
)
ament_export_dependencies(${dependencies})
ament_package()
