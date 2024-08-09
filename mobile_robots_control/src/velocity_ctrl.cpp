/*
@description:
Simple example to command the 4 wheels of the omnidirectional mobile robot using velocity_controller

USAGE:
In different terminals:
$ ros2 launch mobile_robots_control omnibot_velocity_controller_launch.py
	# Launch the omnidirectional mobile robot with 4 wheels
$ ros2 run mobile_robots_control velocity_ctrl
	# Run this node

To see the joint states
$ ros2 topic echo /joint_states
*/
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("velocity_ctrl"); //Node name

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/velocity_controller/commands", 10); //<controller_name>/commands

  RCLCPP_WARN(node->get_logger(), "Node created");

  std_msgs::msg::Float64MultiArray commands; //Create the object

  using namespace std::chrono_literals;
  
  //std::cout << "commands.data size: " << commands.data.size() << std::endl;

  //Initialize data fields with zero 
  int num_joints = 4;
  for (int i=0; i<num_joints; i++) commands.data.push_back(0);
  
  std::cout << "commands.data size: " << commands.data.size() << std::endl;

  commands.data[0] = 3.14;
  commands.data[1] = -1.57; //-------------- constant velocity for the rest of joints during the node lifetime
  commands.data[2] = -1.57;
  commands.data[3] = -1.57;
  publisher->publish(commands);
  RCLCPP_WARN(node->get_logger(), "publishing velocities for 6 s ...");
  std::this_thread::sleep_for(6s);

  commands.data[0] = -3.14;
  publisher->publish(commands);
  RCLCPP_WARN(node->get_logger(), "publishing velocities for 12 s ...");
  std::this_thread::sleep_for(12s);

  //Stop joints
  for (int i=0; i<num_joints; i++) commands.data[i] = 0;

  std::cout << "Stopping wheels" << std::endl;
  publisher->publish(commands);
  std::this_thread::sleep_for(0.5s);
  
  RCLCPP_WARN(node->get_logger(), "Node finished");
  
  node.reset(); //To avoid Error in destruction of rcl subscription handle: Failed to delete datareader
  rclcpp::shutdown();
  return 0;
}

