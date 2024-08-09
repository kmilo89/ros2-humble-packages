/*
@description:
Simple example to command the joints of the manipulator using forward_command_controller

USAGE:
In different terminals:
$ ros2 launch mobile_robots_control omni_manipulator_launch.py
	# Launch the omnidirectional mobile robot with the manipulator composed of 3 joints
$ ros2 run mobile_robots_control omni_position_ctrl
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

  node = std::make_shared<rclcpp::Node>("omni_position_ctrl"); //Node name

  auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/pos_controller/commands", 10); //<controller_name>/commands

  RCLCPP_WARN(node->get_logger(), "Node created");
  std_msgs::msg::Float64MultiArray commands; //Create the object

  using namespace std::chrono_literals;
  //std::cout << "commands.data size: " << commands.data.size() << std::endl;

  //Initialise data fields with zero  
  int num_joints = 3;
  for (int i=0; i<num_joints; i++) commands.data.push_back(0);
  
  //publisher->publish(commands);
  //std::this_thread::sleep_for(1s);
  
  commands.data[0] = -0.3;
  commands.data[1] = -0.7071;
  commands.data[2] = 0.7071;
  publisher->publish(commands);
  std::cout << "publishing pos for 3 s ..." << std::endl;
  std::this_thread::sleep_for(3s);

  commands.data[0] = -0.6;
  commands.data[1] = -1.57;
  commands.data[2] = 1.57;
  publisher->publish(commands);
  std::cout << "publishing pos for 3 s ..." << std::endl;
  std::this_thread::sleep_for(3s);
  
  commands.data[0] = -0.9;
  commands.data[1] = -2.1;
  commands.data[2] = 2.1;
  publisher->publish(commands);
  std::cout << "publishing pos for 3 s ..." << std::endl;
  std::this_thread::sleep_for(3s);

  //Stop joints
  commands.data[0] = 0;
  commands.data[1] = 0;
  commands.data[2] = 0;
  std::cout << "publishing zero pos" << std::endl;
  publisher->publish(commands);
  
  RCLCPP_WARN(node->get_logger(), "Node finished");
  
  node.reset(); //To avoid Error in destruction of rcl subscription handle: Failed to delete datareader
  rclcpp::shutdown();
  return 0;
}
