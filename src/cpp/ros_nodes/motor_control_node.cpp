#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp> // For acknowledgments or status

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode() : Node("motor_control_node")
  {
    // Subscriber to motion commands from the motion planning node
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "motion_commands", 10, std::bind(&MotorControlNode::motion_command_callback, this, std::placeholders::_1));

    // Publisher for sending acknowledgments or status updates
    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_control_status", 10);

    RCLCPP_INFO(this->get_logger(), "Motor Control Node started. Subscribed to /motion_commands, publishing on /motor_control_status");
  }

private:
  void motion_command_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received motion command. Executing low-level motor control...");
    // TODO: Implement low-level motor control logic here
    // This would involve translating the PoseStamped command into joint torques/velocities

    // Publish a status message indicating command received (and potentially execution status)
    auto status_msg = std_msgs::msg::String();
    status_msg.data = "Motion command executed";
    publisher_->publish(status_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotorControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
