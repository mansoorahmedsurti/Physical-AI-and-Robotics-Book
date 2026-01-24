# 🤖 Physical AI & Humanoid Robotics Textbook with RAG Chatbot

![Project Status](https://img.shields.io/badge/Status-Active-success)
![Tech Stack](https://img.shields.io/badge/Stack-Docusaurus%20|%20FastAPI%20|%20RAG-blueviolet)
![License](https://img.shields.io/badge/License-MIT-blue)

> **An AI-Native interactive textbook designed to bridge the gap between digital intelligence and physical humanoid robotics.**

## 📖 About The Project

This project is a cutting-edge educational platform built for the **Panaversity Hackathon I**. It serves as a comprehensive textbook for the *Physical AI & Humanoid Robotics* course, teaching students how to design, simulate, and deploy humanoid robots using ROS 2, Gazebo, and NVIDIA Isaac.

Beyond static content, this project features an embedded **Retrieval-Augmented Generation (RAG) Chatbot**. This AI assistant allows readers to ask questions directly related to the course material, providing instant, context-aware answers grounded in the textbook's content.

### 🌟 Key Features
* **AI-Native Learning:** A Docusaurus-based static site optimized for technical documentation and readability.
* **Integrated RAG Agent:** A smart chatbot that uses vector search (Qdrant) and LLMs to answer student queries instantly.
* **Interactive Syllabus:** Covers modules from ROS 2 Nervous Systems to Vision-Language-Action (VLA) models.
* **Modern Stack:** Built using **Claude Code** and **Spec-Kit Plus** for rapid, spec-driven development.

---

## 🛠️ Tech Stack

### Frontend (The Book)
* **Framework:** [Docusaurus 3.x](https://docusaurus.io/) (React-based SSG)
* **Styling:** Custom CSS / Tailwind
* **Deployment:** Vercel / GitHub Pages

### Backend (The Brain)
* **API:** [FastAPI](https://fastapi.tiangolo.com/) (Python)
* **Database (Relational):** [Neon](https://neon.tech/) (Serverless Postgres)
* **Database (Vector):** [Qdrant](https://qdrant.tech/) (for semantic search)
* **LLM Orchestration:** OpenAI Agents / ChatKit SDKs

---

## 📚 Course Curriculum Covered

This textbook covers the complete "Physical AI" capstone quarter:

1.  **Module 1: The Robotic Nervous System** (ROS 2, Nodes, URDF)
2.  **Module 2: The Digital Twin** (Gazebo Physics, Unity Rendering)
3.  **Module 3: The AI-Robot Brain** (NVIDIA Isaac Sim, Nav2)
4.  **Module 4: Vision-Language-Action (VLA)** (Voice-to-Action, Cognitive Planning)

---

## 🚀 Getting Started

Follow these instructions to set up the project locally.

### Prerequisites
* **Node.js** (v18 or higher)
* **Python** (v3.10 or higher)
* **Git**

### 1. Clone the Repository
```bash
git clone [https://github.com/mansoorahmedsurti/RAG-Chatbot-on-Robotics-Book.git](https://github.com/mansoorahmedsurti/RAG-Chatbot-on-Robotics-Book.git)
cd RAG-Chatbot-on-Robotics-Book
