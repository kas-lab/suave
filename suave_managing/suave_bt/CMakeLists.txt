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

# include_directories(
#   include
# )

add_library(suave_mission SHARED src/suave_bt/suave_mission.cpp)
add_library(is_pipeline_found SHARED src/suave_bt/is_pipeline_found.cpp)
add_library(is_pipeline_inspected SHARED src/suave_bt/is_pipeline_inspected.cpp)
add_library(condition_water_visibility SHARED src/suave_bt/condition_water_visibility.cpp)
add_library(condition_thrusters_ok SHARED src/suave_bt/condition_thrusters_ok.cpp)
add_library(condition_battery_level SHARED src/suave_bt/condition_battery_level.cpp)

add_library(action_arm_thrusters SHARED src/suave_bt/action_arm_thrusters.cpp)
add_library(action_set_guided_mode SHARED src/suave_bt/action_set_guided_mode.cpp)
add_library(action_search_pipeline SHARED src/suave_bt/action_search_pipeline.cpp)
add_library(action_inspect_pipeline SHARED src/suave_bt/action_inspect_pipeline.cpp)
add_library(action_recharge_battery SHARED src/suave_bt/action_recharge_battery.cpp)
add_library(action_change_mode SHARED src/suave_bt/action_change_mode.cpp)


list(APPEND plugin_libs
  suave_mission
  is_pipeline_found
  is_pipeline_inspected
  condition_water_visibility
  condition_thrusters_ok
  condition_battery_level
  action_arm_thrusters
  action_set_guided_mode
  action_search_pipeline
  action_inspect_pipeline
  action_recharge_battery
  action_change_mode
)

foreach(bt_plugin ${plugin_libs})
  # ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_features(${bt_plugin} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE "SUAVE_BT_BUILDING_LIBRARY")
endforeach()

add_executable(suave_bt src/suave_bt.cpp)
ament_target_dependencies(suave_bt ${dependencies})
target_link_libraries(suave_bt ${plugin_libs})

add_executable(suave_bt_extended src/suave_bt_extended.cpp)
ament_target_dependencies(suave_bt_extended ${dependencies})
target_link_libraries(suave_bt_extended ${plugin_libs})

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  suave_bt
  suave_bt_extended
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
