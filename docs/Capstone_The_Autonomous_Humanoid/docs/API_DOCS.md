# Butler Bot API Endpoints and Internal Interfaces Documentation

This document outlines the key API endpoints and internal module interfaces for the Butler Bot system.

## 1. Voice Command Processing (vla_voice_processor.py)

- **Input**: `audio_input` (std_msgs/String) - Simulated audio input.
- **Output**: `voice_commands` (std_msgs/String) - Processed voice commands.

## 2. Task Planning (vla_task_planner.py)

- **Input**: `voice_commands` (std_msgs/String) - Voice commands from `vla_voice_processor.py`.
- **Output**: `high_level_commands` (butler_bot_interfaces/HighLevelCommand) - High-level robot action commands.

## 3. ROS Interface (vla_ros_interface.cpp)

- **Input**: `high_level_commands` (butler_bot_interfaces/HighLevelCommand) - High-level commands from `vla_task_planner.py`.
- **Output**: `cmd_vel` (geometry_msgs/Twist) - Robot velocity commands (for navigation).

## 4. Object Detection (object_detector.py)

- **Input**: Isaac Sim simulation data (internal).
- **Output**: `detected_objects` (std_msgs/String) - Names of detected objects.

## 5. Semantic Mapping (semantic_mapper.py)

- **Input**: `detected_objects` (std_msgs/String) - Detected objects from `object_detector.py`.
- **Input**: `rgb_image` (sensor_msgs/Image) - Simulated RGB image stream.
- **Output**: `semantic_map` (std_msgs/String) - String representation of the semantic map.

## 6. Grasping Control (grasping_controller.cpp)

- **Input**: Target pose (geometry_msgs/Pose) for inverse kinematics.
- **Input**: Gripper command (boolean) for open/close.
- **Output**: Joint angles (std::vector&lt;double&gt;) from IK solution.

## 7. Grasp Integration (vla_grasp_integrator.py)

- **Input**: `high_level_commands` (butler_bot_interfaces/HighLevelCommand) - Grasp commands.
- **Input**: `detected_objects` (std_msgs/String) - Confirmation of detected objects.
- **Input**: `object_poses` (geometry_msgs/PoseStamped) - Poses of detected objects.
- **Output**: (Internal calls to GraspingController)

## 8. VSLAM (vslam_node.py)

- **Input**: Isaac Sim sensor data (internal, simulated).
- **Output**: `vslam_pose` (geometry_msgs/PoseStamped) - Robot's pose.
- **Output**: `vslam_odometry` (nav_msgs/Odometry) - Robot's odometry.