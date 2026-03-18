# 🦛 Kinematic-Aware Improved Hippo Optimization (IHO) for Robot Path Planning

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Status](https://img.shields.io/badge/Status-SCI%20Under%20Review-orange.svg)]()

> Official implementation of the paper: **"Kinematic-Aware Improved Hippo Optimization with Laplacian Ironing for Swarm-based Path Planning in Cluttered Environments"**.

---

### 🌐 [English Version](README.md) | [中文版](README_zh.md)

---

### 💡 Key Innovations

1.  **Kinematic-Aware Constraint**: Integrated into the search mechanism, eliminating infeasible paths with sharp turns, ensuring robot-executable trajectories.
2.  **Laplacian Ironing Operator**: A novel topological tension-release mechanism applied in late-stage search, smoothing waypoints to achieve **cliff-like convergence**.

---

### 🏗️ Proposed Framework

This project establishes a stringent, multi-dimensional evaluation system across five complex maps.

<p align="center">
  <img src="assets/Evaluation_Framework.png" width="100%" alt="Evaluation Framework">
  <br>
  <em>Fig. 1. The comprehensive multidimensional benchmark testing framework for IHO.</em>
</p>

---

### 🎥 Hardware Demonstration

Validation on a physical mobile robot platform across distinct challenging environments.

<p align="center">
  <video src="hardware_demos/map_5.mov" width="70%" controls autoplay loop muted></video>
  <br>
  <em>Demo: Large-scale, Highly Constrained Labyrinth (Map 5). Note the smooth navigation without sharp turns.</em>
</p>

---

### 📊 Simulation Results

Statistical comparison with state-of-the-Art (SOTA) algorithms. IHO guarantees **100% collision safety** with extreme efficiency.

<p align="center">
  <img src="results/planned_paths/Path_Map4.png" width="48%" alt="Path Map 4">
  <img src="results/convergence_curves/Convergence_Map4.png" width="48%" alt="Convergence Map 4">
  <br>
  <em>Comparison on Map 4: Our IHO (blue line) achieves a smooth, taut path and unique cliff-like convergence (right plot, blue line dropping at step 70).</em>
</p>

---

### 📂 Repository Structure

```text
Kinematic-Aware-IHO/
├── src/                      # Source code for algorithms and environment
│   ├── main.m                # Main execution script
│   ├── IHO_Planner.m         # Proposed Kinematic-Aware IHO
│   ├── HO_Planner.m          # Original Hippo Optimization
│   └── ...                   # Other SOTA planners (PSO, GWO, etc.)
├── results/                  # Generated paths and convergence curves
├── assets/                   # High-res figures for the paper
└── hardware_demos/           # Real-world robot navigation videos
