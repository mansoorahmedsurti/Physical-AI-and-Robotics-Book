#ifndef ROBOT_HAL_H
#define ROBOT_HAL_H

#include <vector>

namespace butler_bot_hal
{

class RobotHAL
{
public:
  RobotHAL();
  ~RobotHAL();

  void setMotorVelocity(double linear_x, double angular_z);
  void setGripperState(bool open);
  void getJointStates(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& efforts);
};

} // namespace butler_bot_hal

#endif // ROBOT_HAL_H
