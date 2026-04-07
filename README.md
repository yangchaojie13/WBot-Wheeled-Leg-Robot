# 🤖 WBot: Two-Wheeled Inverted Pendulum Robot

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?style=flat&logo=ros)
![C++](https://img.shields.io/badge/C++-17-00599C?style=flat&logo=c%2B%2B)
![Gazebo](https://img.shields.io/badge/Simulation-Gazebo-FFB71B?style=flat)
![Status](https://img.shields.io/badge/Status-LQR_Sim_Success-brightgreen)

WBot 是一台基于 **ROS 2 (Humble)** 和 **LQR (线性二次型调节器)** 算法开发的双轮倒立摆（轮腿）机器人。本项目包含了从底层硬件设计、下位机固件到底层动力学推导、再到高级仿真与控制的完整全栈开发流程。

> **Note:** 本项目目前已在 Gazebo 仿真环境中完美实现了 LQR 闭环自平衡控制，正在向真机 (Sim-to-Real) 迁移中。

---

## 📂 项目架构 (Project Structure)

为了保持工程的清晰与模块化，本项目采用标准的机器人全栈开发目录结构：

- **`ros2_ws/`** : ROS 2 核心控制工作空间 (C++)
  - `wbot_description/`: 机器人的 URDF 模型与网格文件 (STL)
  - `wbot_control/`: 基于动力学推导的 LQR 核心控制节点与 PD 关节控制器
- **`hardware/`** : 包含机器人的 SolidWorks 3D 模型与机械结构图纸
- **`firmware/`** : 下位机 (GD32/STM32) 的电机驱动与传感器 (IMU) 读取代码
- **`simulation/`** : 基于拉格朗日方程的 MATLAB 动力学推导脚本与 LQR $K$ 矩阵解算器

---

## ✨ 核心特性 (Features)

* **精确的物理建模**：通过拉格朗日方程推导了非线性的倒立摆动力学模型，并精确提取了机身与轮子的惯性张量 (Inertia Tensor)。
* **LQR 最优控制**：抛弃玄学的纯手动 PID 调参，使用 LQR 算法通过状态反馈（位移、速度、倾角、角速度）实现全局最优平衡。
* **抗积分饱和与安全机制**：在 C++ 控制节点中加入了自适应积分限幅与“电子手刹”保护机制，防止在 IMU 数据丢失或跌倒时发生数值爆炸。
* **ROS 2 现代架构**：完全基于 ROS 2 的 Topic 通信体系开发，为未来的 `ros2_control` 硬件接口和 `Nav2` 自动导航打下了基础。

---

## 🚀 快速启动仿真 (Quick Start: Simulation)

如果你想在本地的 Gazebo 中运行 WBot 的平衡仿真，请确保你已安装 `ROS 2 Humble` 和 `Gazebo`。

## 1. 编译工作空间

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. 启动 Gazebo 并生成机器人

```bash
# 在终端 1 启动 Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

```bash
# 在终端 2 召唤机器人 (设置 z=0.3 确保从半空安全着陆)
ros2 run gazebo_ros spawn_entity.py -entity wbot -file src/wbot_description/urdf/robot.urdf -z 0.3
```

## 3. 在终端 3 运行核心平衡算法

```bash
ros2 run wbot_control lqr_controller
```

## 🗺️ 未来路线图 (Roadmap)

- [x] 完成 URDF 模型构建与 Gazebo 物理参数映射
- [x] 完成双轮倒立摆动力学推导与 MATLAB LQR 矩阵解算
- [x] C++ 控制节点实现与闭环平衡测试
- [ ] 添加 cmd_vel 订阅，实现键盘/手柄遥控移动
- [ ] 编写 ros2_control 硬件接口，打通 Sim-to-Real 鸿沟
- [ ] 加入激光雷达与里程计 (Odom) 融合，测试 2D SLAM 建图
- [ ] 部署 Nav2 导航栈，实现自主避障寻路

## 🤝 交流与贡献

欢迎任何对双轮腿足机器人感兴趣的开发者提交 Issue 或 Pull Request！
