# 🦛 Kinematic-Aware Improved Hippo Optimization (IHO) for Robot Path Planning

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Status](https://img.shields.io/badge/Status-SCI%20Under%20Review-orange.svg)]()

> Official implementation of the paper: **"Kinematic-Aware Improved Hippo Optimization with Laplacian Ironing for Swarm-based Path Planning in Cluttered Environments"**.

---

### 🌐 [English Version] | [中文版](#-动力学感知改进河马优化算法iho机器人路径规划)

---

### 💡 Key Innovations

1.  **Kinematic-Aware Constraint**: Integrated into the search mechanism, eliminating infeasible paths with sharp turns, ensuring robot-executable trajectories.
2.  **Laplacian Ironing Operator**: A novel topological tension-release mechanism applied in late-stage search, smoothing waypoints to achieve **cliff-like convergence**.

---

### 🏗️ Proposed Framework

This project establishes a stringent, multi-dimensional evaluation system across five complex maps.

<p align="center">
  <img src="https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO_NAME/main/assets/Evaluation_Framework.png"width="100%" alt="Evaluation Framework">
  <br>
  <em>Fig. 1. The comprehensive multidimensional benchmark testing framework for IHO.</em>
</p>

---

### 🎥 Hardware Demonstration

Validation on a physical mobile robot platform across distinct challenging environments.

<p align="center">
  <video src="https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO_NAME/main/hardware_demos/demo_map5_compressed.mp4" width="70%" controls autoplay loop muted></video>
  <br>
  <em>Demo: Large-scale, Highly Constrained Labyrinth (Map 5). Note the smooth navigation without sharp turns.</em>
</p>

---

### 📊 Simulation Results

Statistical comparison with state-of-the-Art (SOTA) algorithms. IHO guarantees **100% collision safety** with extreme efficiency.

<p align="center">
  <img src="https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO_NAME/main/results/planned_paths/Path_Map4.png" width="48%" alt="Path Map 4">
  <img src="https://raw.githubusercontent.com/YOUR_USERNAME/YOUR_REPO_NAME/main/results/convergence_curves/Convergence_Map4.png" width="48%" alt="Convergence Map 4">
  <br>
  <em>Comparison on Map 4: Our IHO (blue line) achieves a smooth, taut path and unique cliff-like convergence (right plot, blue line dropping at step 70).</em>
</p>

---

### 💻 How to Use

1.  Clone this repository: `git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git`
2.  Open **MATLAB (R2023b or later)**.
3.  Navigate to the `src/` directory.
4.  Run `main.m` to generate the default simulation results for all maps.

---
