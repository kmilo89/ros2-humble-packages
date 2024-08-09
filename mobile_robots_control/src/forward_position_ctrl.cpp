/*
@description:
Simple example to command joints of a robot using forward_command_controller

USAGE:
In different terminals:
$ ros2 launch mobile_robots_control forward_position_launch.py
	# Launch the simulated robot with 2 joints
$ ros2 run mobile_robots_control forward_position_ctrl
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

  node = std::make_shared<rclcpp::Node>("forward_position_ctrl"); //Node name

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/my_position_controller/commands", 10); //<controller_name>/commands

  RCLCPP_WARN(node->get_logger(), "Node created");

  std_msgs::msg::Float64MultiArray commands; //Create the object

  using namespace std::chrono_literals;
  
  //std::cout << "commands.data size: " << commands.data.size() << std::endl;

  //Initialise data fields with zero  
  int num_joints = 2;
  for (int i=0; i<num_joints; i++) commands.data.push_back(0);
  
  std::cout << "commands.data size: " << commands.data.size() << std::endl;

  publisher->publish(commands);
  std::this_thread::sleep_for(1s);
  
  commands.data[0] = 0.1; //prismatic (m)
  commands.data[1] = -0.7071; //revolute (rad)
  publisher->publish(commands);
  std::cout << "publishing pos for 4 s ..." << std::endl;
  std::this_thread::sleep_for(4s);

  commands.data[0] = 0.2;
  commands.data[1] = 0.7071;
  publisher->publish(commands);
  std::cout << "publishing pos for 3 s ..." << std::endl;
  std::this_thread::sleep_for(3s);

  //Stop joints
  commands.data[0] = 0;
  commands.data[1] = 0;
  std::cout << "publishing zero pos" << std::endl;
  publisher->publish(commands);
  //std::this_thread::sleep_for(2s);
  
  RCLCPP_WARN(node->get_logger(), "Node finished");
  
  node.reset(); //To avoid Error in destruction of rcl subscription handle: Failed to delete datareader
  rclcpp::shutdown();
  return 0;
}
