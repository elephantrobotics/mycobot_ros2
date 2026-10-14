cmake_minimum_required(VERSION 3.22)
project(mycobotpro630_gazeboros2)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "srv/Pro630Rpc.srv")
ament_export_dependencies(rosidl_default_runtime)

add_library(pro630_current_start_adapter SHARED src/use_current_start_state.cpp)
target_compile_features(pro630_current_start_adapter PUBLIC cxx_std_17)
ament_target_dependencies(pro630_current_start_adapter
  moveit_ros_planning moveit_core pluginlib rclcpp)
pluginlib_export_plugin_description_file(moveit_core planning_request_adapters.xml)
install(TARGETS pro630_current_start_adapter LIBRARY DESTINATION lib)



if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.pyc" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(FILES ARCHITECTURE.md DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/pro630_gui_backend.py
  scripts/pro630_control_gui.py
  scripts/activate_gazebo_controllers.py
  scripts/pro630_slider_gui.py
  scripts/pro630_pose_gate.py
  scripts/pro630_environment_state.py
  scripts/pro630_control_session.py
  scripts/pro630_drag_follow.py
  scripts/pro630_robot_bridge.py
  scripts/pro630_collision_scene.py
  scripts/slider_control_gazebo.py
  scripts/verify_initial_pose.py
  scripts/teleop_keyboard_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/pro630_configuration.py scripts/pro630_model.py scripts/pro630_hold_profile.py scripts/pro630_real_keyboard.py
  scripts/pro630_session_support.py
  scripts/pro630_drag_support.py
  scripts/pro630_public_client.py
  scripts/pro630_broker_support.py
  scripts/pro630_real_mirror.py
  scripts/pro630_sdk_adapter.py scripts/pro630_feedback.py
  scripts/pro630_collision_check.py
  scripts/pro630_gripper_control.py scripts/pro630_scene_support.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
