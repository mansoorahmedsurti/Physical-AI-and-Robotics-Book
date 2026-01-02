---
id: module-1-ros2
title: "Module 1: The Robotic Nervous System (ROS 2)"
sidebar_label: Module 1 - ROS 2
---

# Module 1: The Robotic Nervous System (ROS 2)

## 1. Introduction: The Nervous System of Robotics

Just as a biological nervous system transmits signals between the brain (planning) and the muscles (actuation), a robot needs a robust middleware to communicate between its software components. **Robot Operating System 2 (ROS 2)** is that nervous system.

In this module, we move away from monolithic codebases—where one error crashes the whole robot—and adopt a **Distributed Node Architecture**.

### Why ROS 2? (vs. ROS 1)
For the "Physical AI" era, we use ROS 2 because of three critical upgrades:
1.  **DDS (Data Distribution Service):** Unlike ROS 1, which relied on a central "Master" node (a single point of failure), ROS 2 uses industry-standard DDS for decentralized communication.
2.  **Real-Time Capabilities:** Essential for safety-critical tasks like humanoid balancing, where a millisecond delay can cause a fall.
3.  **Python 3 First:** Native support for modern AI libraries (PyTorch, TensorFlow) which we will need for our VLA modules.



## 2. Key Concepts: The Anatomy of Communication

To build our "Butler Bot," you must master these four primitives:

* **Nodes:** Independent executable processes. Think of them as individual neurons. One node reads the camera, another controls the wheels.
* **Topics:** The "veins" of the system. Nodes publish data to topics (e.g., `/camera/image_raw`) or subscribe to them. This is **asynchronous** (Fire-and-forget).
* **Services:** A **synchronous** two-way handshake. "I request X, you give me Y." (e.g., "Spawn a new object" or "Reset Simulation").
* **Actions:** Designed for long-running tasks. "Go to the kitchen." This takes time, so the robot provides feedback ("I am 50% there") and allows cancellation ("Stop!").

## 3. Implementation: Setting Up Your Reflex Arc

We will implement a basic "reflex arc" to demonstrate real-time interaction. This mimics a biological withdrawal reflex: **Stimulus (Sensor) → Processing → Response (Motor).**

### Step 1: Environment Setup
First, we verify our ROS 2 installation. Open your terminal and source the underlay:

```bash
source /opt/ros/jazzy/setup.bash
# Verify installation
printenv | grep ROS