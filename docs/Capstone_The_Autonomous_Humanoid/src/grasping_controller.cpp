#include "grasping_controller.h"
#include <iostream>
#include <cmath> // For mathematical functions like atan2, sqrt

namespace butler_bot_grasping
{

GraspingController::GraspingController()
: link1_length_(0.3), link2_length_(0.3), link3_length_(0.1) // Example link lengths
{
  std::cout << "Grasping Controller initialized." << std::endl;
}

GraspingController::~GraspingController()
{
  std::cout << "Grasping Controller shut down." << std::endl;
}

bool GraspingController::solveInverseKinematics(const geometry_msgs::msg::Pose& target_pose, std::vector<double>& joint_angles)
{
  std::cout << "Solving inverse kinematics for target pose: (" << target_pose.position.x << ", " << target_pose.position.y << ", " << target_pose.position.z << ")" << std::endl;

  // This is a highly simplified 2D inverse kinematics for a 2-DOF arm for demonstration.
  // A real robot arm would require a more complex 3D solution.

  double x = target_pose.position.x;
  double y = target_pose.position.y;

  double d_squared = x*x + y*y;
  double d = std::sqrt(d_squared);

  if (d > (link1_length_ + link2_length_))
  {
      std::cerr << "Target out of reach!" << std::endl;
      return false; // Target unreachable
  }

  double cos_q2 = (d_squared - link1_length_*link1_length_ - link2_length_*link2_length_) / (2 * link1_length_ * link2_length_);

  if (cos_q2 > 1.0 || cos_q2 < -1.0)
  {
      std::cerr << "No solution for Q2 (invalid cos_q2 value)." << std::endl;
      return false;
  }

  double q2_solution1 = std::atan2(std::sqrt(1 - cos_q2*cos_q2), cos_q2); // Elbow up
  double q2_solution2 = std::atan2(-std::sqrt(1 - cos_q2*cos_q2), cos_q2); // Elbow down

  // For simplicity, let's pick one solution (e.g., elbow up)
  double q2 = q2_solution1;

  double sin_q1_num = y * (link1_length_ + link2_length_*cos_q2) - x * (link2_length_*std::sin(q2));
  double cos_q1_num = x * (link1_length_ + link2_length_*cos_q2) + y * (link2_length_*std::sin(q2));

  double q1 = std::atan2(sin_q1_num, cos_q1_num);

  joint_angles.clear();
  joint_angles.push_back(q1);
  joint_angles.push_back(q2);
  joint_angles.push_back(0.0); // Assuming a third joint for gripper rotation, fixed for now

  std::cout << "IK solution: q1 = " << q1 << ", q2 = " << q2 << std::endl;
  return true;
}

void GraspingController::commandGripper(bool open)
{
  std::cout << "Commanding gripper: " << (open ? "open" : "closed") << std::endl;
  // In a real system, this would send commands to the gripper actuator.
}

} // namespace butler_bot_grasping
