/*
@description:
This node subscribes to the "/odom" topic to obtain the velocity of the robot.
Taking into account the kinematic model of the omnidirectional robot composed of 4 
mecanum wheels, the corresponding rotational velocity of each wheel is published in 
the "/velocity_controller/commands" topic.

@author: C. Mauricio Arteaga-Escamilla from "Robotica Posgrado (YouTube channel)"
*/
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

//Robot parameters
float Lx = 0.3, Ly = 0.26, R = 0.1; //Distances with respect to the robot frame and wheels radius

using namespace std::chrono_literals;
using std::placeholders::_1;

class Omni4wheels : public rclcpp::Node
{
  public:
    Omni4wheels() : Node("omni_4wheels")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
      
      subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&Omni4wheels::odom_callback, this, _1));
      
      //Initialize data fields with zero 
      int num_wheels = 4;
      for (int i=0; i<num_wheels; i++) commands.data.push_back(0);
      
      RCLCPP_WARN(this->get_logger(), "Node has been initialized");
    }
    
    ~Omni4wheels() //Class destructor definition
    {
      RCLCPP_WARN(this->get_logger(), "Node has been terminated"); 
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      double vx, vy, wz; //Local variables
      
      //Get the robot velocities
      vx = msg->twist.twist.linear.x;
      vy = msg->twist.twist.linear.y;
      wz = msg->twist.twist.angular.z;
      //cout << "Vx: " << vx << "Vy: " << vy << "W: " << wz << endl;

      //Compute the velocity of each mecanum wheel
      commands.data[0] = (vx-vy-(Lx+Ly)*wz)/R; //w1
      commands.data[1] = (vx+vy+(Lx+Ly)*wz)/R; //w2
      commands.data[2] = (vx-vy+(Lx+Ly)*wz)/R; //w3
      commands.data[3] = (vx+vy-(Lx+Ly)*wz)/R; //w4
      
      publisher_->publish(commands);
    }

  private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    std_msgs::msg::Float64MultiArray commands;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Omni4wheels>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}


