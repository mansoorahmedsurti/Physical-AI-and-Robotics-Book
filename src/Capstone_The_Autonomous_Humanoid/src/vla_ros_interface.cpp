#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "butler_bot_interfaces/msg/high_level_command.hpp"
#include "butler_bot_hal/robot_hal.h"

class VLARosInterface : public rclcpp::Node
{
public:
  VLARosInterface()
  : Node("vla_ros_interface_node"), robot_hal_()
  {
    subscription_ = this->create_subscription<butler_bot_interfaces::msg::HighLevelCommand>(
      "high_level_commands", 10, std::bind(&VLARosInterface::high_level_command_callback, this, std::placeholders::_1));

    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "VLA ROS Interface Node Initialized.");

    // TODO: Performance optimization efforts for vision, navigation, and grasping
    // would be focused here, e.g., using more efficient algorithms, reducing data
    // copies, or offloading computation to specialized hardware. This is a placeholder
    // to indicate where such work would occur.
  }

private:
  void high_level_command_callback(const butler_bot_interfaces::msg::HighLevelCommand::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received high-level command: %s, %s, %s",
                msg->command_type.data.c_str(), msg->target_object.data.c_str(), msg->target_location.data.c_str());

    if (msg->command_type.data == "NAVIGATE")
    {
      RCLCPP_INFO(this->get_logger(), "Processing NAVIGATION command to %s", msg->target_location.data.c_str());
      // For now, simulate movement, eventually integrate with Nav2
      geometry_msgs::msg::Twist twist_msg;
      twist_msg.linear.x = 0.2; // Example linear velocity
      twist_msg.angular.z = 0.0; // Example angular velocity
      cmd_vel_publisher_->publish(twist_msg);
      // TODO: Add error handling for HAL command failures
      robot_hal_.setMotorVelocity(twist_msg.linear.x, twist_msg.angular.z);
    }
    else if (msg->command_type.data == "GRASP")
    {
      RCLCPP_INFO(this->get_logger(), "Processing GRASP command for %s", msg->target_object.data.c_str());
      // Simulate grasping action
      robot_hal_.setGripperState(true); // Open gripper
      // In a real scenario, this would involve perception and precise manipulation
      robot_hal_.setGripperState(false); // Close gripper
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unknown command type: %s", msg->command_type.data.c_str());
    }
  }

  rclcpp::Subscription<butler_bot_interfaces::msg::HighLevelCommand>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  butler_bot_hal::RobotHAL robot_hal_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VLARosInterface>());
  rclcpp::shutdown();
  return 0;
}
