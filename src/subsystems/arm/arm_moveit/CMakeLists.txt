cmake_minimum_required(VERSION 3.22)
project(arm_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
ament_target_dependencies(hello_moveit rclcpp moveit_ros_planning_interface)


install(TARGETS
  hello_moveit
  DESTINATION lib/${PROJECT_NAME})

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config srdf
DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
