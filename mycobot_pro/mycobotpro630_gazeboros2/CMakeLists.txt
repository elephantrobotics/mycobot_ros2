cmake_minimum_required(VERSION 3.22)
project(mycobotpro630_gazeboros2)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/slider_control_gazebo.py
  scripts/follow_display_gazebo.py
  scripts/teleop_keyboard_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

