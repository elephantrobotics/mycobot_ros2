cmake_minimum_required(VERSION 3.22)
project(mycobotpro450_gazeboros2)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(pro450_current_start_adapter SHARED src/use_current_start_state.cpp)
target_compile_features(pro450_current_start_adapter PUBLIC cxx_std_17)
ament_target_dependencies(pro450_current_start_adapter
  moveit_ros_planning moveit_core pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core planning_request_adapters.xml)
install(TARGETS pro450_current_start_adapter LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pro450_current_start test/test_pro450_current_start.cpp)
  ament_target_dependencies(test_pro450_current_start
    moveit_ros_planning moveit_core pluginlib rclcpp)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/activate_gazebo_controllers.py
  scripts/coords_broadcaster.py
  scripts/pro450_slider_gui.py
  scripts/pro450_pose_gate.py
  scripts/slider_control_gazebo.py
  scripts/verify_initial_pose.py
  scripts/follow_display_gazebo.py
  scripts/teleop_keyboard_gazebo.py
  scripts/calibrate_pro450_hold_speed.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/pro450_hold_profile.py scripts/pro450_real_keyboard.py
  scripts/pro450_real_mirror.py
  scripts/pro450_sdk_adapter.py scripts/pro450_feedback.py
  scripts/pro450_gripper_profile.py scripts/pro450_gripper_speed.json
  DESTINATION lib/${PROJECT_NAME})

ament_package()
