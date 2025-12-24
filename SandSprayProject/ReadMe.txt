# 基于工艺机理的复杂曲面喷砂路径规划与优化系统
# (Path Planning and Optimization for Complex Surface Sandblasting Based on Process Mechanism)

## 1. 项目简介 (Introduction)

本项目是基于 Siemens NX (NX Open API) 开发的定制化喷砂机器人离线编程系统。针对热冲压门环等复杂曲面零件，系统实现了从特征识别、区域重构、路径生成到全局序列优化的全流程自动化。

核心创新点在于结合了**喷砂工艺机理（切向剥离力模型）**，通过算法自动调整喷枪姿态，并引入**遗传算法 (Genetic Algorithm)** 解决多路径间的空移优化问题，实现了高效、均匀的喷砂覆盖。

---

## 2. 核心功能模块 (Key Features)

### 2.1 智能特征识别与分区 (Feature Recognition & Partitioning)
* **代码模块**: `FaceProcessor`, `RegionBuilder`, `GeometricProcessFaceClassification`
* **功能**:
    * 自动提取曲面最小曲率半径与法向特征。
    * 将复杂拓扑面分类为：**水平面 (Horizontal)**、**侧壁 (Sidewall)**、**凸面 (Convex)**、**凹面 (Concave)**。
    * 基于 BFS (广度优先搜索) 算法进行区域拓扑重构，自动识别闭环边界。

### 2.2 变姿态覆盖路径生成 (Variable Attitude Path Generation)
* **代码模块**: `RegionCurveBuilder`, `SprayCurveBuilder`
* **功能**:
    * **边界引导**: 基于区域边界生成样条导引线 (Spline Guide)。
    * **自适应切片**: 在保证重叠率的前提下自适应调整步距，解决边缘覆盖不足问题。
    * **智能连接**: 自动处理断开的路径段，通过 `ConnectPathPointsWithRingSupport` 算法生成连续轨迹。

### 2.3 全局序列优化 (Global Sequence Optimization)
* **代码模块**: `SprayPathSequencer`, `GeneticAlgorithm`
* **功能**:
    * 将多区域路径排序建模为广义旅行商问题 (Generalized TSP)。
    * 实现了基于**遗传算法 (GA)** 的求解器：
        * **编码**: 路径索引 + 方向 (正/反)。
        * **算子**: 锦标赛选择、顺序交叉 (OX)、多类型变异。
        * **局部改良**: 引入 2-Opt 算法优化局部解。
    * **效果**: 显著减少机器人空移距离 (Air-move Distance)。

### 2.4 工艺机理导向的姿态控制与导出 (Process-Oriented Attitude Control)
* **代码模块**: `CSVExporter`
* **功能**:
    * **位姿自适应**: 根据第 2 章工艺仿真结论，自动计算最优喷射角度：
        * 侧壁区域：强制侧倾 $\pm 60^\circ$。
        * 凹凸区域：强制侧倾 $\pm 45^\circ$。
    * **6-DOF 导出**: 输出包含坐标 $(x, y, z)$ 和工具矢量 $(i, j, k)$ 的标准 CSV 轨迹文件，可直接用于机器人控制器。

### 2.5 喷砂效果闭环评估 (Closed-loop Evaluation)
* **代码模块**: `SprayEffectEvaluator`, `SprayEffectEvalutatorDialog`
* **功能**:
    * 基于高斯分布能量场模型计算表面覆盖率。
    * 生成覆盖率热力图（紫红/深绿/黄/红）。
    * **缺陷重规划**: 自动聚类覆盖不足区域 (Bad Area Clustering)，支持一键生成补充路径。

---

## 3. 系统环境与依赖 (Requirements)

* **开发环境**: Visual Studio 2019 / 2022
* **NX 版本**: Siemens NX 12.0 或更高版本 (测试于 NX 2306)
* **编程语言**: C++ 11/14
* **核心依赖**:
    * NX Open C++ API (`libnxopencpp`, `libufun`, etc.)
    * Standard Template Library (STL)

---

## 4. 项目结构 (Project Structure)

```text
SandSprayProject/
├── Source/
│   ├── Dialog/                  # NX Block UI Styler 对话框逻辑
│   │   ├── GeometricProcessFaceClassification.cpp  # 特征分类UI
│   │   ├── SprayCurveBuilder.cpp                   # 路径生成UI
│   │   ├── SpraySequencerandCSVOutputDialog.cpp    # 排序与导出UI
│   │   └── SprayEffectEvalutatorDialog.cpp         # 仿真评估UI
│   │   
│   ├── ToolClass/               # 核心算法实现
│   │   ├── FaceProcessor.cpp    # 面分类算法
│   │   ├── RegionBuilder.cpp    # 区域构建算法
│   │   ├── RegionCurveBuilder.cpp # 路径与切片算法
│   │   ├── GeneticAlgorithm.cpp   # 遗传算法求解器
│   │   ├── SprayPathSequencer.cpp # 序列管理器
│   │   ├── SprayEffectEvaluator.cpp # 覆盖率评估
│   │   └── CSVExporter.cpp        # 变姿态导出逻辑
│   │   
│   └── Entry/
│       └── SandSprayProject.cpp   # 插件入口点 (ufusr)
├── Headers/                     # 头文件 (.h, .hpp)
└── UI/                          # DLX 对话框文件
