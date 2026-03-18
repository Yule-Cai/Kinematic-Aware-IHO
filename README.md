# 🦛 Kinematic-Aware Improved Hippo Optimization (IHO) for Robot Path Planning

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Status](https://img.shields.io/badge/Status-Under%20Review-orange.svg)]()

> Official implementation of the paper:
> **"Kinematic-Aware Improved Hippo Optimization with Laplacian Ironing for Swarm-based Path Planning in Cluttered Environments"** *(under review)*

---

### 🌐 Language

[English](README.md) | [中文](README_zh.md)

---

## 💡 Key Contributions

### 1. Kinematic-Aware Constraint
A kinematic-aware mechanism is embedded into the swarm optimization process to explicitly handle nonholonomic constraints of mobile robots. This eliminates physically infeasible paths (e.g., in-place turns or sharp-angle segments), ensuring that generated trajectories are directly executable on real robotic platforms.

### 2. Laplacian Ironing Operator
Inspired by geometric signal processing, we propose a Laplacian Ironing Operator that smooths waypoint distributions during late-stage optimization. This operator induces a distinctive **cliff-like convergence** behavior, significantly improving path smoothness without sacrificing optimality.

---

## 🏗️ Benchmark Framework

We establish a comprehensive evaluation framework consisting of five challenging environments with varying scales and topologies:
* Small-scale narrow corridor maps ($40 \times 40$)
* Large-scale cluttered maze environments ($80 \times 80$)

All methods are evaluated under a strict collision penalty:
$$\lambda_{static} = 10^6$$
ensuring a zero-tolerance safety criterion.

<p align="center">
  <img src="assets/Evaluation_Framework.png" width="100%" alt="Evaluation Framework">
  <br>
  <em>Fig. 1. Multi-dimensional benchmark framework and comparison matrix.</em>
</p>

---

---

## 🎥 Hardware Validation

The proposed method is validated on a real-world mobile robotic platform across multiple complex environments.

<p align="center">
  <img src="hardware_demos/demo_map_5.jpg" width="70%" alt="Hardware Demonstration Map 5">
  <br>
  <em>Fig. 2. Large-scale constrained maze (Map 5). The mobile robot navigates through a high-occupancy labyrinth using the optimized path from Kinematic-Aware IHO. Note the smooth trajectory in narrow corridors without any physically infeasible sharp turns.</em>
</p>

---

## 📊 Results and Comparisons

We compare IHO with state-of-the-art algorithms including HO (baseline), SBOA, ARO, INFO, PSO, and GWO.

### Key Findings:
* **100% collision-free** solutions even with small population size ($N = 30$)
* Superior path smoothness and compactness
* Clear late-stage convergence acceleration induced by Laplacian ironing

<p align="center">
  <img src="results/planned_paths/Path_Map4.png" width="48%" alt="Path">
  <img src="results/convergence_curves/Convergence_Map4.png" width="48%" alt="Convergence">
  <br>
  <em>Map 4 comparison: IHO (blue) achieves smoother paths and exhibits cliff-like convergence behavior.</em>
</p>

---

## 📂 Project Structure

```text
Kinematic-Aware-IHO/
├── src/                    # Core algorithms and environments
│   ├── main.m              # Entry point
│   ├── IHO_Planner.m       # Proposed IHO algorithm
│   ├── HO_Planner.m        # Original HO algorithm
│   └── ...                 # Other baselines (PSO, GWO, etc.)
├── results/                # Generated paths and convergence curves
├── assets/                 # Figures used in the paper
└── hardware_demos/         # Real robot demonstrations
```

---

## ⚙️ Requirements

* OS: Windows 10/11, Ubuntu 20.04+, or macOS
* MATLAB: R2023b or later (recommended)
* Toolboxes: **None required** (fully reproducible using base MATLAB)

---

## 🚀 Quick Start

```bash
git clone https://github.com/Yule-Cai/Kinematic-Aware-IHO.git
```

Then:

1. Open MATLAB
2. Navigate to the `src/` directory
3. Run:

```matlab
main.m
```

---

## 📄 License

This project is licensed under the **CC BY-NC-SA 4.0 License**.

© 2026 Yule Cai



