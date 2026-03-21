# 📑 Experiment Log: Implementation of Adaptive Dynamic Penalty Mechanism

### 1. Overall Objectives
To address the path planning deadlock and statistical anomalies in complex environments (e.g., Map 4 and Map 5), an **Adaptive Dynamic Penalty** mechanism was introduced. This approach ensures a balance between 100% physical safety (collision-free) and optimal path quality.

### 2. Core Algorithmic Modifications

#### 2.1 Adaptive Dynamic Penalty Function
The traditional static "death penalty" was replaced with a quintic model that evolves with the current iteration $t$:
$$Penalty(t) = 500 + 99500 \times \left(\frac{t}{T_{max}}\right)^5$$



* **Early Stage:** The low penalty weight allows the swarm to "traverse" obstacles to discover the global optimal topology and escape local minima.
* **Late Stage:** The penalty weight increases quintically, strictly forcing all paths to converge into 100% safe, collision-free regions.

#### 2.2 Experimental Fairness (Standardized Evaluation)
To ensure a rigorous "Apples-to-Apples" comparison, the evaluation function `EvaluatePath` has been standardized across all baseline algorithms (HO, PSO, GWO, SBOA, ARO, INFO). All algorithms now compete under the same kinematic constraints and penalty logic.

### 3. Key Experimental Findings

* **Absolute Dominance in Map 4:**
    Ours (IHO) achieved a **100.0% Success Rate (SR)**, maintaining a clean trajectory without any collision penalties, significantly outperforming competitors in both safety and path length.
* **Breakthrough in Map 5:**
    While the original HO algorithm failed completely (SR: 0.0%) in this bottleneck map, the modified IHO successfully discovered high-quality feasible paths (Best: 123.40).

### 4. Data Output Description
The final statistical results are automatically exported to: **`Table.xlsx`**.
* **Success Rate (SR):** Measures the algorithm's survivability and reliability under complex constraints.
* **Valid Metrics:** Mean, Worst, and Std are calculated strictly from 100% successful runs, adhering to the statistical conventions of top-tier robotics journals.

---

# 📑 实验日志：自适应动态惩罚机制的实现

### 1. 总体目标
针对复杂环境（如 Map 4 和 Map 5）下的路径规划死锁及统计异常问题，引入了**自适应动态惩罚**机制。该方法旨在确保 100% 物理安全性（无碰撞）与最优路径质量之间的平衡。

### 2. 核心改进记录

#### 2.1 自适应动态惩罚函数
将传统的静态“致死惩罚”替换为随迭代次数 $t$ 演化的五次方模型：
$$Penalty(t) = 500 + 99500 \times \left(\frac{t}{T_{max}}\right)^5$$

* **前期 (Early Stage)：** 较低的惩罚权重允许种群“穿透”障碍物，以探索全局最优拓扑路径并跳出局部最优陷阱。
* **后期 (Late Stage)：** 惩罚权重呈五次方爆发式增长，强制所有路径回归 100% 安全的无碰撞区域。

#### 2.2 实验公平性保障 (统一评价标准)
为了确保严谨的“同台竞技（Apples-to-Apples）”，所有对比算法（HO, PSO, GWO, SBOA, ARO, INFO）的评价函数 `EvaluatePath` 已全部统一。所有算法均在相同的动力学约束和惩罚逻辑下运行。

### 3. 关键实验发现

* **Map 4 的绝对优势：**
    Ours (IHO) 算法实现了 **100.0% 的成功率 (SR)**，在没有任何碰撞惩罚的情况下保持了纯净的轨迹，在安全性及路径长度上均显著优于对比算法。
* **Map 5 的破局能力：**
    在原版 HO 算法于此瓶颈地图中全军覆没（成功率: 0.0%）的情况下，改进后的 IHO 成功找到了高质量的可行路径（最优值: 123.40）。

### 4. 数据产出说明
最终实验统计结果已自动导出为：**`Table.xlsx`**。
* **Success Rate (SR)：** 衡量算法在复杂约束下的生存能力和可靠性。
* **Valid Metrics：** 均值 (Mean)、最差值 (Worst) 和标准差 (Std) 仅针对 100% 成功的运行轮次进行计算，符合机器人学顶级期刊的统计惯例。

---
**Author:** Yule Cai  
**Date:** 2026-03-21  
