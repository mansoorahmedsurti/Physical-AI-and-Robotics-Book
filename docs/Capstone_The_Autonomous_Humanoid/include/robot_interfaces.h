#ifndef ROBOT_INTERFACES_H
#define ROBOT_INTERFACES_H

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace butler_bot_interfaces
{

// Enum for different robot states
enum RobotState
{
  IDLE,
  NAVIGATING,
  GRASPING,
  SPEAKING,
  ERROR
};

// Example: A simple command message for high-level actions
struct HighLevelCommand
{
  std_msgs::msg::String command_type;
  std_msgs::msg::String target_object;
  std_msgs::msg::String target_location;
};

// Example: Robot status message
struct RobotStatus
{
  RobotState current_state;
  std_msgs::msg::String status_message;
};

} // namespace butler_bot_interfaces

#endif // ROBOT_INTERFACES_H
