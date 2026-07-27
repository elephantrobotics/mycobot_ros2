cmake_minimum_required(VERSION 3.22)
project(ultraarm_p1_moveit2)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(ultraarm_p1_moveit2_coupling_adapter SHARED
  src/coupling_planning_request_adapter.cpp
)
ament_target_dependencies(ultraarm_p1_moveit2_coupling_adapter
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp
)
target_compile_features(ultraarm_p1_moveit2_coupling_adapter PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(moveit_core coupling_adapter_plugin.xml)

install(TARGETS ultraarm_p1_moveit2_coupling_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES coupling_adapter_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
