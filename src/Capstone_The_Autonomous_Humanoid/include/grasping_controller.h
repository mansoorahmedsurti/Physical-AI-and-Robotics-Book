#ifndef GRASPING_CONTROLLER_H
#define GRASPING_CONTROLLER_H

#include <vector>
#include "geometry_msgs/msg/pose.hpp"

namespace butler_bot_grasping
{

class GraspingController
{
public:
  GraspingController();
  ~GraspingController();

  // Solves inverse kinematics to find joint angles for a target pose
  bool solveInverseKinematics(const geometry_msgs::msg::Pose& target_pose, std::vector<double>& joint_angles);

  // Commands the gripper to open or close
  void commandGripper(bool open);

private:
  // Simulated robot arm parameters (for a simple 3-DOF arm example)
  double link1_length_;
  double link2_length_;
  double link3_length_;
};

} // namespace butler_bot_grasping

#endif // GRASPING_CONTROLLER_H
