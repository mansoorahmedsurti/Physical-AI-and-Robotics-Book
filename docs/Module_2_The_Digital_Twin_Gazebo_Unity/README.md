---
id: module-2-digital-twin
title: Module 2: The Digital Twin (Gazebo & Unity)
sidebar_label: Module 2 - Digital Twin
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Introduction to Digital Twins

This module explores the concept of a **"Digital Twin"** in robotics—a virtual replica of a physical system that mirrors its behavior, physics, and interactions. In the era of Physical AI, we cannot afford to test dangerous or complex algorithms on expensive hardware immediately. Digital twins are crucial for developing, testing, and refining AI-driven robotic behaviors in a safe, cost-effective, and reproducible environment.



We will leverage two powerful simulation platforms: **Gazebo** for realistic physics and sensor modeling, and **Unity** for advanced visual rendering and scene creation.

## Gazebo: Physics and Sensor Simulation

Gazebo is the de facto standard simulator for the ROS ecosystem. It interacts with ROS 2 just like a real robot would—publishing sensor data to topics and subscribing to motor commands.

### 1. Realistic Physics Engines
Gazebo does not just "animate" movement; it calculates it. It uses robust physics engines (like **ODE**, **Bullet**, or **Dart**) to accurately simulate:
* **Gravity & Inertia:** How heavy is the robot? Will it tip over?
* **Collisions:** What happens when the arm hits a table?
* **Friction & Contact:** Will the wheels slip on this surface?
This is essential for ensuring that robot behaviors developed in simulation translate effectively to the real world (Sim-to-Real transfer).

### 2. Sensor Modeling (The "Eyes" of the Robot)
A robot is blind without sensors. Gazebo allows us to attach virtual sensors to our robot that mimic real hardware. We will configure:
* **LiDAR (Light Detection and Ranging):** For mapping rooms and avoiding obstacles.
* **Depth Cameras (RGB-D):** For detecting 3D objects (like the Intel RealSense).
* **IMUs:** For tracking orientation and acceleration.

These sensors publish data to standard ROS 2 topics (e.g., `/scan`, `/camera/image_raw`), allowing your navigation stack to run unchanged on both the simulator and the real robot.



### 3. Robot Models (URDF vs. SDF)
To simulate a robot, we must define it mathematically.
* **URDF (Unified Robot Description Format):** An XML format used by ROS to describe the robot's structure (links) and movement (joints). It creates the "kinematic tree."
* **SDF (Simulation Description Format):** An XML format native to Gazebo that includes extra details like friction coefficients, mass, and simulation-specific plugins.



## Unity: Advanced Visuals and Interactive Environments

While Gazebo excels at physics, **Unity** is a game engine that excels at photorealism. For "Vision-Language-Action" (Module 4) models, the AI needs to see the world as humans do.

### 1. High-Fidelity Rendering
Unity allows us to design virtual worlds with realistic lighting, shadows, textures, and particle effects. This is critical for training computer vision models, which might fail if a simulation looks too "cartoonish."

### 2. The Unity-ROS 2 Bridge
We will use the **ROS-TCP-Connector** package to link Unity and ROS 2.
* **ROS 2 Side:** Runs the control logic (Nav2, MoveIt).
* **Unity Side:** Renders the robot and environment, sending sensor data back to ROS 2 over TCP/IP.
This hybrid approach combines Gazebo's physics accuracy (or Unity's own PhysX) with Unity's graphical prowess.



## Implementation Steps: Spawning Your First Robot

In this module, we will execute the following workflow:

1.  **Define the Robot:** Write a `.urdf` file describing a simple 2-wheeled mobile robot with a caster wheel.
2.  **Launch Gazebo:** Use a ROS 2 launch file (`ros2 launch`) to start the simulator server.
3.  **Spawn Entity:** Inject the robot model into the empty Gazebo world.
4.  **Drive:** Use the `teleop_twist_keyboard` node to drive the virtual robot using your keyboard, verifying that the wheels spin and the physics engine calculates friction correctly.

## Code Snippet: A Simple URDF Link
Here is what a "Link" (a physical part of the robot) looks like in code:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
    <material name="blue"/>
  </visual>
  <collision>
    <geometry>
      <box size="0.5 0.3 0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
</link>