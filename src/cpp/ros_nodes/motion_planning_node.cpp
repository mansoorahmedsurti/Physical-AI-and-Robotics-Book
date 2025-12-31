#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

class MotionPlanningNode : public rclcpp::Node
{
public:
  MotionPlanningNode() : Node("motion_planning_node")
  {
    // Subscriber to voice commands
    voice_command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "voice_commands", 10, std::bind(&MotionPlanningNode::voice_command_callback, this, std::placeholders::_1));

    // Subscriber to detected objects from vision processing
    vision_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "detected_objects", 10, std::bind(&MotionPlanningNode::vision_callback, this, std::placeholders::_1));

    // Publisher for robot motion commands
    motion_command_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("motion_commands", 10);

    RCLCPP_INFO(this->get_logger(), "Motion Planning Node started. Subscribed to /voice_commands and /detected_objects, publishing on /motion_commands");
  }

private:
  void voice_command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received voice command: '%s'", msg->data.c_str());
    // TODO: Parse voice command and translate into motion goal
    // Example: if command is "move to kitchen", set goal accordingly
  }

  void vision_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received detected object: '%s'", msg->data.c_str());
    // TODO: Use detected object information to inform motion planning
  }

  void plan_and_execute_motion(const geometry_msgs::msg::PoseStamped& goal_pose)
  {
    // TODO: Implement motion planning algorithm (e.g., MoveIt) to generate trajectory
    // For now, just publish the goal pose as a dummy command
    motion_command_publisher_->publish(goal_pose);
    RCLCPP_INFO(this->get_logger(), "Published motion command.");
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr vision_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr motion_command_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanningNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
