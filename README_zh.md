# 🦛 动力学感知改进河马优化算法 (IHO) 机器人路径规划

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b+-blue.svg)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)
[![Status](https://img.shields.io/badge/Status-SCI%20Under%20Review-orange.svg)]()

> 本仓库为以下论文的官方代码实现：
> **"Kinematic-Aware Improved Hippo Optimization with Laplacian Ironing for Swarm-based Path Planning in Cluttered Environments"** (正在审稿中).

---

### 🌐 [English Version](README.md) | [中文版](README_zh.md)

---

### 💡 核心理论创新

1.  **动力学感知约束 (Kinematic-Aware Constraint)**：将底盘非完整约束无缝融入群智能寻优机制，剔除包含“原地掉头”或“锐角折线”的物理不可行路径，确保生成的轨迹可直接部署于真实的移动机器人或四足机器人控制系统。
2.  **拉普拉斯熨斗算子 (Laplacian Ironing Operator)**：受几何信号处理启发，创新性地提出拓扑张力释放机制。在算法寻优后期对空间航点进行平滑，实现独特的**断崖式收敛 (Cliff-like convergence)**。

---

### 🏗️ 综合实验评估体系

本项目构建了一个包含 5 张不同尺度、不同拓扑特征的严苛环境基准测试库（$40\times40$ 狭窄走廊与 $80\times80$ 大尺度复杂迷宫），并在极其严苛的零容忍碰撞惩罚函数（$\lambda_{static}=10^6$）下进行公平测试。

<p align="center">
  <img src="assets/Evaluation_Framework.png" width="100%" alt="实验评估流程图">
  <br>
  <em>图 1. IHO 综合多维度基准测试框架与算法竞争矩阵。</em>
</p>

---

### 🎥 真实场景实机验证 (Hardware Demonstration)

我们在物理移动机器人平台上，横跨多个极具挑战性的障碍物环境进行了实机验证。

<p align="center">
  <video src="hardware_demos/map_5.mov" width="70%" controls autoplay loop muted></video>
  <br>
  <em>演示：大尺度高约束迷宫环境 (Map 5)。请注意机器人穿越复杂走廊时的极度平滑性与安全性。</em>
</p>

---

### 📊 仿真结果与性能对比

与基础算法（HO）以及其他 5 种目前最新的顶级群智能算法（SBOA, ARO, INFO, PSO, GWO）进行了定量统计与定性动态对比。结果表明，IHO 在极低种群规模（$N=30$）下依然保证了 **100% 的全局零碰撞绝对安全**。

<p align="center">
  <img src="results/planned_paths/Path_Map4.png" width="48%" alt="地图4轨迹图">
  <img src="results/convergence_curves/Convergence_Map4.png" width="48%" alt="地图4收敛曲线">
  <br>
  <em>复杂环境 Map 4 对比：我们的 IHO 算法（蓝线）不仅实现了最紧致平滑的路径，且在其收敛曲线后期（右图，约第 70 代）清晰地展现了由拉普拉斯熨斗算子激发的“断崖式下跌”。</em>
</p>

---

### 📂 仓库目录结构

```text
Kinematic-Aware-IHO/
├── src/                      # 算法与环境源代码
│   ├── main.m                # 主运行脚本
│   ├── IHO_Planner.m         # 本文提出的 IHO 算法核心代码
│   ├── HO_Planner.m          # 原始河马优化算法
│   └── ...                   # 其他对比算法 (PSO, GWO 等)
├── results/                  # 跑图生成的轨迹与收敛曲线
├── assets/                   # 论文中使用的高清架构图
└── hardware_demos/           # 真实机器人导航演示视频
</p>
---

### ⚙️ 环境依赖

* **操作系统**: Windows 10/11, Ubuntu 20.04+, 或 macOS。
* **软件环境**: MATLAB R2023b 或更高版本（推荐，以获得最佳的画图渲染效果）。
* **附加工具箱**: 无需安装任何额外的特定工具箱。代码仅依赖 MATLAB 基础函数，以确保 100% 的可复现性。
</p>
---

### 💻 快速开始 (How to Use)

1. 克隆本仓库到本地：
```bash
git clone [https://github.com/Yule-Cai/Kinematic-Aware-IHO.git](https://github.com/Yule-Cai/Kinematic-Aware-IHO.git)

打开 MATLAB。

将工作路径切换至本项目的 src/ 文件夹。

直接运行 main.m 脚本。
</p>
© 2026. 本项目遵循 CC BY-NC-SA 4.0 开源协议。
