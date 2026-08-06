# rgb_qr_detect

基于 ROS1 + OpenCV 的 RGB 色块圆形检测与二维码识别节点。打开本地摄像头，实时检测画面中心区域内的红/绿/蓝圆形目标，并识别二维码内容，通过 ROS 话题发布结果。

## 功能

- **颜色圆形检测**：在画面中心 25% 区域内检测红 / 绿 / 蓝圆形目标
- **二维码识别**：使用 OpenCV `QRCodeDetector` 解码二维码
- **坐标解析**：若二维码内容为 `(x,y)` 格式，额外发布目标坐标
- **可视化窗口**：实时显示检测框、圆心与二维码边框（按 `q` 或 `Esc` 退出）

## 依赖

| 依赖 | 说明 |
|------|------|
| ROS1 | Melodic / Noetic（catkin） |
| OpenCV | 需支持 `QRCodeDetector`、V4L2 |
| 摄像头 | V4L2 设备（默认 `/dev/video0`） |

ROS 包依赖：`roscpp`、`std_msgs`、`sensor_msgs`、`cv_bridge`、`image_transport`

> 当前节点直接通过 OpenCV 打开摄像头，未订阅图像话题；`cv_bridge` / `image_transport` 为包声明依赖。

## 目录结构

```
rgb_qr_detect/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── rgb_qr_detect.launch
└── src/
    └── detect.cpp
```

## 编译

将本包装入 catkin 工作空间后编译：

```bash
cd ~/catkin_ws/src
# 把 rgb_qr_detect 包放到 src 下
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

先启动 ROS master，再启动节点：

```bash
roscore
# 另开终端
roslaunch rgb_qr_detect rgb_qr_detect.launch
```

或直接运行可执行文件：

```bash
rosrun rgb_qr_detect detect
```

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/detected_color` | `std_msgs/Int32` | 颜色代码：`0` 红、`1` 绿、`2` 蓝、`-1` 未检测到 |
| `/qr_data` | `std_msgs/String` | 二维码原始字符串 |
| `/target_x` | `std_msgs/Int32` | 从二维码解析的 X（仅当内容匹配 `(x,y)`） |
| `/target_y` | `std_msgs/Int32` | 从二维码解析的 Y（仅当内容匹配 `(x,y)`） |

查看示例：

```bash
rostopic echo /detected_color
rostopic echo /qr_data
```

## 摄像头与处理参数

代码中默认配置：

- 设备：`cv::VideoCapture(0, cv::CAP_V4L2)`
- 采集分辨率：1920×1080，MJPEG，30 FPS
- 处理宽度：缩放到 640 像素宽后再做检测
- 循环频率：约 30 Hz

可按板卡性能在 `src/detect.cpp` 中调整分辨率、处理宽度或 `ros::Rate`。

## ARM 板卡

本项目无 x86 / CUDA 专用代码，可在 aarch64 / armhf 板卡（树莓派、RK3588、Jetson 等）上编译运行，需满足：

1. 已安装对应架构的 ROS1 与 OpenCV
2. 摄像头驱动正常（V4L2）
3. 有图形界面或 X11 转发时可视化窗口才可用；无头环境需去掉 `imshow` / `waitKey` 或使用虚拟显示

弱算力板卡建议降低采集分辨率或处理宽度，以保证帧率。

## 许可证

MIT
