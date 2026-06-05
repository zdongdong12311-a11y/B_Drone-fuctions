# Step 2: EGO-Planner 局部避障与轨迹规划

本步骤负责利用 FAST-LIO2 输出的高频里程计和 Livox Mid-360 的点云数据，通过 EGO-Planner 算法进行局部的三维轨迹规划和动态避障。

| 项目 | 内容 |
|------|------|
| Linux 版本 | Ubuntu 20.04 |
| ROS 版本 | Noetic |
| 机载电脑 | Orange Pi 5 Max |
| 激光雷达 | Livox Mid-360 |
| 规划算法 | EGO-Planner |

---

## 一、模块功能

EGO-Planner 是一个轻量级且高效的基于梯度的局部路径规划器。本项目中使用它来：
1. 接收来自 FAST-LIO2 的高频里程计 `/Odom_high_freq`。
2. 接收来自 LiDAR 的全局/局部注册点云 `/cloud_registered`。
3. 根据目标点 (2D Nav Goal 或全局航点)，在考虑运动学约束下生成安全的、无碰撞的 B-spline 轨迹。
4. 轨迹服务器 (`traj_server`) 解析轨迹并下发控制指令。

---

## 二、编译 EGO-Planner

如果尚未编译该模块，可以在工作空间中进行编译。通常情况下，只需将本目录拷贝或链接到 ROS 的 workspace `src` 下：

```bash
sudo apt-get install libarmadillo-dev
mkdir ego_ws/src
cd ego_ws/src
git clone https://github.com/ZJU-FAST-Lab/ego-planner.git
cd ..
catkin_make
# 将 2.ego-planner 的内容复制或移动到 src/ 下
cp -r C:/Users/z1597/Desktop/Highly_modular_autonomous_drone-ego/2.ego-planner/plan_manage/launch ~/ego_ws/src/ego-planner/launch
cd ~/ego_ws
catkin_make
source ~/ego_ws/devel/setup.bash
```

---

## 三、配置与运行

### 1. 核心运行脚本

主要使用实机运行的 launch 文件来启动 EGO-Planner 和相关服务：

```bash
roslaunch ego_planner single_run_in_exp.launch
```

### 2. 关键参数说明 (`single_run_in_exp.launch` / `advanced_param_exp.xml`)

| 参数 / 话题映射 | 说明 |
|-----------------|------|
| `odom_topic` | 默认映射为 `/Odom_high_freq`，由 FAST-LIO2 提供的高频里程计 |
| `cloud_topic` | 默认映射为 `/cloud_registered`，三维环境点云数据 |
| `max_vel` | 设定无人机的最大飞行速度 (默认 1.0 m/s) |
| `max_acc` | 设定无人机的最大加速度 (默认 2.0 m/s²) |
| `planning_horizon` | 规划视界大小，建议设置为传感器感知范围的 1.5 倍 (默认 6) |
| `flight_type` | `1`: 使用 RViz 的 2D Nav Goal 设定目标点<br>`2`: 使用预设的全局航点 |

### 3. 注意事项

- **坐标系一致性**：确保输入的点云和里程计处于同一个父坐标系（例如 `map` 或 `camera_init`），避免规划器获取到错误的环境信息导致撞机。
- **计算资源**：EGO-Planner 虽然高效，但在复杂的 3D 环境中高频规划仍可能占用一定的 CPU 资源。可以根据实机运行表现适当调节点云降采样频率或控制频率。
- **与控制层的接口**：`traj_server` 节点会将规划好的轨迹（`/planning/bspline`）转换成控制指令（如 `/position_cmd` 或相应的 MAVROS setpoint），请确保下游控制器的订阅话题相匹配。

---

## 四、配置 shell 环境

按需将环境加载写入 `~/.bashrc`：

```bash
source ~/ego_ws/devel/setup.bash --extend
```
