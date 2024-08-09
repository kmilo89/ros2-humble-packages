cmake_minimum_required(VERSION 3.5)
project(mobile_robots_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(ament_cmake_python REQUIRED) #for python scripts
find_package(rclcpp REQUIRED)
#find_package(rclpy REQUIRED) #for python scripts
#find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED) #for ros control
find_package(rclcpp_action REQUIRED) #for ros control actions


set(dependencies "rclcpp" "std_msgs")

add_executable(velocity_ctrl src/velocity_ctrl.cpp)
ament_target_dependencies(velocity_ctrl ${dependencies})

add_executable(forward_position_ctrl src/forward_position_ctrl.cpp)
ament_target_dependencies(forward_position_ctrl ${dependencies})

add_executable(omni_position_ctrl src/omni_position_ctrl.cpp)
ament_target_dependencies(omni_position_ctrl ${dependencies})

add_executable(joint_position_trayectory src/joint_position_trayectory.cpp)
ament_target_dependencies(joint_position_trayectory
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(omni_4wheels src/omni_4wheels.cpp)
ament_target_dependencies(omni_4wheels ${dependencies} nav_msgs)


install(TARGETS
  velocity_ctrl
  forward_position_ctrl
  joint_position_trayectory
  omni_position_ctrl
  omni_4wheels
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch urdf meshes rviz worlds config src scripts
  DESTINATION share/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables #They must start with the shebag tag
install(PROGRAMS
  scripts/omni_teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME} )

ament_package()


