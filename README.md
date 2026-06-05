# Highly Modular Autonomous Drone

基于 **Livox Mid-360 激光雷达** 的模块化无人机自主避障、建图与导航系统。支持多航点自主飞行、视觉识别抓取投放、多层安全保护机制。

## 硬件配置

| 组件 | 型号 | 备注 |
|------|------|------|
| 机载电脑 | Orange Pi 5 Max | RK3588, 8GB+ RAM |
| 飞控 | CUAV X7+ | PX4 1.13.3 |
| 激光雷达 | Livox Mid-360 | 非重复扫描，360° 视场 |
| 摄像头 (可选) | USB 1080p | MJPG, 用于视觉识别 |
| 舵机爪 (可选) | 铝合金铁爪 + ESP8266 | 串口控制，抓取/投放 |

## 软件环境

| 组件 | 版本 |
|------|------|
| OS | Ubuntu 20.04 (aarch64) |
| ROS | Noetic |
| PX4 | 1.13.3 |
| Python | 3.8+ |
| OpenCV | 4.x (视觉识别模块) |



## 系统架构

```
Livox Mid-360 LiDAR
    │
    ├─→ Livox-SDK2 → livox_ros_driver2 → 原始点云
    │
    ├─→ FAST-LIO2 → /Odometry & /Odom_high_freq
    │       │
    │       ├─→ lidar_to_mavros → /mavros/vision_pose/pose → PX4 EKF2 (视觉位置融合)
    │       │
    │       └─→ /cloud_registered (去畸变三维点云)
    │               │
    │               └─→ EGO-Planner (局部3D避障与轨迹规划)
    │                       │
    │                       └─→ traj_server (轨迹跟踪) → 飞控指令
    │                               │
    │                               └─→ navigation.py → 航点任务 / 控制逻辑
    │
    └─→ MAVROS ↔ PX4 (飞控通信)
```

## 安全机制

本项目包含多层次安全保护:

| 保护类型 | 触发条件 | 响应动作 |
|----------|----------|----------|
| **低电量降落** | 电池 < 20% (可配置) | 立即降落 |
| **连接断开保护** | MAVROS 断连 | 紧急降落 |
| **位姿超时保护** | 3s 无位姿更新 | 紧急降落 |
| **航点超时跳转** | 单航点 > 120s (可配置) | 跳转下一航点 |
| **起飞超时保护** | 起飞 > 30s 未达目标高度 | 切换 AUTO.LAND |
| **Ctrl+C 安全降落** | 用户中断 | 优雅降落而非直接退出 |
| **速度限幅** | 规划器发出超速指令 | 钳位到安全速度 |

> **首次飞行前务必手持测试**，确认位姿无漂移后再解锁飞行。

## 坐标变换 (TF) 树

```
camera_init (FAST-LIO2 实时估计)
    │
    └─→ body (PX4 / MAVROS 发布 / FAST-LIO2 里程计子坐标系)
```

| 变换 | 发布者 | 说明 |
|------|--------|------|
| `camera_init` → `body` | FAST-LIO2 | SLAM 实时估计 |
| `base_link` → `body` | PX4 (MAVROS) | 飞控本体位姿 |

> **重要**: EGO-Planner 需要统一的环境坐标系，确保规划器接收的点云和里程计位于同一坐标系下。

## 目录结构

```
drone/
├── README.md                    ← 本文档
├── navigation.py                ← 自主导航脚本 (纯导航)
├── point.txt                    ← 航点文件 (支持 # 注释)
│
├── 1.mid360-drone/              ← Step 1: LiDAR 驱动 + 状态估计 + PX4 桥接
│   ├── README.md                ← 详细安装指南
│   └── lidar_to_mavros/         ← ROS 包: FAST-LIO2 → PX4 vision_pose 桥接
│       ├── launch/lidar_to_mavros.launch
│       ├── src/lidar_to_mavros.cpp
│       ├── CMakeLists.txt
│       └── package.xml
│
├── 2.ego-planner/               ← Step 2: 3D 轨迹规划与避障 (EGO-Planner)
│   ├── README.md                ← 详细配置指南
│   └── plan_manage/             ← 规划器核心启动与配置文件
│       └── launch/
│           ├── single_run_in_exp.launch
│           └── advanced_param_exp.xml
│
└── Modular_fuctions/            ← 模块化功能集
    ├── opecv_RGB_舵机控制铝合金铁爪/   ← OpenCV 颜色识别 + 舵机爪
    │   ├── opencv_nav_micro.py
    │   ├── R.png / G.png / B.png
    │   └── README.md
    └── rknn-yolov8-master/      ← RK3588 NPU 加速 YOLOv8
        ├── src/                 ← C++ 源码 (三线程流水线)
        ├── launch/
        ├── weights/
        └── README.md
```

## 前置安装

每个步骤的详细安装指南见对应目录的 README:

1. **[1.mid360-drone/README.md](./1.mid360-drone/README.md)** — Mid-360 配网与驱动
   - Livox-SDK2 安装
   - livox_ros_driver2 编译
   - FAST-LIO2 安装 (需修改源码适配 driver2)
   - lidar_to_mavros 编译
   - PX4 EKF2 参数调优

2. **[2.ego-planner/README.md](./2.ego-planner/README.md)** — 3D 轨迹规划与避障
   - EGO-Planner 编译与配置
   - 规划器参数详解

3. **[Modular_fuctions/](./Modular_fuctions/)** — 扩展功能
   - OpenCV 颜色识别 + 舵机爪控制
   - RK3588 NPU 加速 YOLOv8 目标检测

## 快速启动

完成前置安装后:

```bash
# 1. 确保 ROS 环境已 source或写入bashrc
source /opt/ros/noetic/setup.bash
source ~/livox_ws/devel/setup.bash --extend
source ~/fast_lio2_ws/devel/setup.bash --extend
source ~/trans_ws/devel/setup.bash --extend
source ~/ego_ws/devel/setup.bash --extend

source ~/.bashrc

# 2. 在终端运行自主导航脚本 
python3 navigation.py              # 纯航点导航
```

### ROS 参数配置

导航脚本支持通过 ROS 参数服务器动态配置 (无需改代码):

```bash
# 通过命令行参数覆盖默认值
python3 navigation.py _takeoff_height:=1.0 _waypoint_timeout:=60.0
```

#### 核心参数说明

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `takeoff_height` | 0.8 | 默认起飞高度 (m) |
| `kp_z` | 1.5 | 高度 P 增益 |
| `kd_z` | 0.0 | 高度 D 增益 (抑制振荡) |
| `max_xy_speed` | 1.5 | 最大水平速度 (m/s) |
| `max_z_speed` | 0.8 | 最大垂直速度 (m/s) |
| `waypoint_xy_tol` | 0.3 | 到达航点 XY 容差 (m) |
| `waypoint_timeout` | 120.0 | 单航点超时 (s) |
| `low_battery_threshold` | 20.0 | 低电量阈值 (%) |
| `waypoint_file` | 自动搜索 | 航点文件路径 |


## 航点文件格式

编辑 `point.txt`，每行一个航点:

```
# 格式: x y z hover_time
# x          - 目标点 X 坐标 (米, 对应全局坐标系)
# y          - 目标点 Y 坐标 (米, 对应全局坐标系)
# z          - 目标高度 (米, 相对起飞点)
# hover_time - 到达后悬停时间 (秒)

0.0  0.0  0.6  2.0
1.0  0.0  0.8  3.0
1.0  1.0  1.0  3.0
0.0  1.0  0.8  3.0
0.0  0.0  0.6  2.0
```

航点文件搜索优先级:
1. ROS 参数 `~waypoint_file`
2. 环境变量 `DRONE_WAYPOINT_FILE`
3. 脚本同目录 `point.txt`
4. `~/point.txt`

## 架构说明与已知限制

### 3D 轨迹规划 (EGO-Planner)

本项目采用 EGO-Planner 替代了传统的 2D 激光切片方案。EGO-Planner 直接处理 FAST-LIO2 的三维点云 (`/cloud_registered`) 进行真 3D 避障规划。

**优势**:
- 实现真正的全方位三维避障，能够应对复杂高低起伏的障碍物。
- 生成的 B-spline 轨迹符合无人机动力学，飞行更平滑。

**已知限制与安全建议**:
- **计算开销**：3D 规划比 2D 方案计算量更大，在复杂的狭窄环境中，RK3588 可能会有一定的性能压力。
- **动态障碍物**：EGO-Planner 具备一定的动态避障能力，但对于高速移动物体可能反应不及。
- **视场角**：虽然 Mid-360 是 360 度雷达，但无人机的移动方向优先，规划时会综合利用视界范围内的数据。

### FAST-LIO2 坐标系说明

官方 hku-mars FAST-LIO2 的坐标系名硬编码在 C++ 源码中:
- 父坐标系: `camera_init`
- 子坐标系: `body`

如需修改，需直接编辑 `src/FAST_LIO/src/laserMapping.cpp` 中约 5 处硬编码字符串，修改后重新编译。

### PX4 EKF2 参数

| 参数 | 建议 | 说明 |
|------|------|------|
| `EKF2_EV_CTRL` | 启用水平/垂直位置和偏航融合 | 开启视觉位置融合 |
| `EKF2_HGT_MODE` | Vision | 高度源使用视觉 |
| `EKF2_GPS_CTRL` | 关闭 GPS 融合 | 室内/无 GPS 场景 |
| `EKF2_EV_DELAY` | 实测 | Mid-360 + FAST-LIO2 延迟需现场测 |
| `EKF2_EV_POS_X/Y/Z` | 实际安装外参 | LiDAR 相对飞控中心的偏移 |
| `EKF2_EVP_NOISE` | 配合调优 | 视觉位置噪声 |
| `EKF2_EVA_NOISE` | 配合调优 | 视觉姿态噪声 |

延迟测量方法:

```bash
rostopic delay /mavros/vision_pose/pose
rostopic delay /mavros/local_position/pose
```

## 自主导航脚本详解

### `navigation.py` — 纯航点导航

执行流程:
1. 连接 MAVROS，等待飞控就绪
2. 切换 OFFBOARD 模式并解锁
3. 自动起飞到指定高度
4. 读取 `point.txt` 航点
5. 逐航点导航 (将目标点发送给 EGO-Planner 或由脚本执行控制)
6. 到达每个航点后悬停指定时长
7. 全部完成后安全降落

安全特性:
- 低电量监测 → 自动降落
- MAVROS 断连检测 → 紧急降落
- 位姿超时检测 (>3s 无更新) → 紧急降落
- Ctrl+C → 优雅安全降落
- 航点超时 → 自动跳转下一航点

## 安全检查清单

飞行前请逐项确认:

- [ ] 机载电脑与飞控串口连接正常 (`/dev/ttyUSB0` 可访问)
- [ ] Mid-360 网络连接正常 (`ping 192.168.1.1xx` 通)
- [ ] 所有 ROS 包已编译并 source (无 `rospack find` 报错)
- [ ] 手持测试: 位姿无显著漂移 (移动一圈回起点误差 < 10cm)
- [ ] 遥控器失控保护 (Failsafe) 已配置
- [ ] 电池满电 (> 90%)
- [ ] EKF2_EV_DELAY 已实测并配置
- [ ] 航点文件 `point.txt` 已准备且格式正确

## 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| EGO-Planner 报无点云或规划失败 | TF 树不完整或里程计无高频输出 | 检查 `/Odom_high_freq` 和 `/cloud_registered` 话题 |
| 起飞后漂移 | OFFBOARD 前设定点未锁位 | 已在代码中修复: 锁死 lock_x/lock_y/lock_yaw |
| 爪子串口连接失败 | 权限不足或设备不存在 | `sudo chmod 666 /dev/ttyUSB0` |
| FAST-LIO2 编译找不到 driver2 | CMAKE_PREFIX_PATH 未设置 | `export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/livox_ws/devel` |
