cmake_minimum_required(VERSION 3.8)
project(robot_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(DIRECTORY 
  launch urdf meshes rviz worlds scripts
  DESTINATION share/${PROJECT_NAME})

  install(PROGRAMS 
    scripts/cmd_vel_pub.py scripts/odom_sub.py
    DESTINATION lib/${PROJECT_NAME})

ament_package()
