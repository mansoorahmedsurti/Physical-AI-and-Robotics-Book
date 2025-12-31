Book on "Physical AI & Humanoid Robotics"

Target Audience: BSCS/STEM students. Requires basic Python & Linear Algebra.
Focus: "Embodied Intelligence" — Controlling Humanoid Robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac.

Success Criteria:
- **Code Verification:** 100% of code must execute in **ROS 2 Jazzy (LTS)** and **Isaac Sim 5.0**.
- **Hardware Check:** Explicitly warns readers they need NVIDIA RTX GPUs (RTX 3070 or higher recommended).
- **VLA Integration:** Demonstrates LLMs controlling robot hardware (Voice-to-Action).
- **Citations:** IEEE Style [1]. Mix of official docs (80%) and key papers (20%).

Constraints:
- **Structure:** 4 Core Modules + Capstone (Matches the Quarter Overview).
- **Visuals:** Heavy use of Architecture Diagrams (Nodes/Topics) over generic photos.
- **Repository Standard:** One folder per Module (e.g., `/src/module_1_nervous_system/`).

Detailed Outline:

## Introduction
- Setup: Ubuntu 22.04/24.04, ROS 2 Jazzy, Drivers.
- Concept: The "Physical AI Stack".

## Module 1: The Robotic Nervous System (ROS 2)
- **Focus:** Middleware & Hardware Abstraction.
- **Key Concepts:** Nodes, Topics, Services, URDF (Robot Description).
- **Project:** "The Reflex Arc" (Python Agents controlling a URDF leg).

## Module 2: The Digital Twin (Gazebo & Unity)
- **Focus:** Physics & Interaction.
- **Key Concepts:** Gazebo Sim (Gravity/Collision) vs. Unity (Visuals/HRI).
- **Project:** Building a simulation environment with simulated LiDAR/Depth sensors.

## Module 3: The AI-Robot Brain (NVIDIA Isaac)
- **Focus:** Perception & Navigation.
- **Key Concepts:** Synthetic Data Generation, VSLAM, Nav2 Stack (Path Planning).
- **Project:** Autonomous navigation of a humanoid in a mapped room.

## Module 4: Vision-Language-Action (VLA)
- **Focus:** Cognitive Layer.
- **Key Concepts:** OpenAI Whisper (Voice) + LLM Logic = Robot Action.
- **Project:** "Clean the Room" Agent (Voice command -> Planning -> Execution).

## Capstone: The Autonomous Humanoid
- **Goal:** Full system integration.
- **Deliverable:** "Butler Bot" (Voice -> Nav -> Vision -> Grasping).