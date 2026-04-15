cmake_minimum_required(VERSION 3.5)
project(roboticsgroup_upatras_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(roboticsgroup_upatras_gazebo_mimic_joint_plugin SHARED src/mimic_joint_plugin.cpp)
target_include_directories(roboticsgroup_upatras_gazebo_mimic_joint_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
)
ament_target_dependencies(roboticsgroup_upatras_gazebo_mimic_joint_plugin control_toolbox gazebo_dev rclcpp)

add_library(roboticsgroup_upatras_gazebo_disable_link_plugin SHARED src/disable_link_plugin.cpp)
target_include_directories(roboticsgroup_upatras_gazebo_disable_link_plugin PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
)
ament_target_dependencies(roboticsgroup_upatras_gazebo_disable_link_plugin gazebo_dev)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS
    roboticsgroup_upatras_gazebo_mimic_joint_plugin
    roboticsgroup_upatras_gazebo_disable_link_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  roboticsgroup_upatras_gazebo_mimic_joint_plugin
  roboticsgroup_upatras_gazebo_disable_link_plugin
)

ament_package()
