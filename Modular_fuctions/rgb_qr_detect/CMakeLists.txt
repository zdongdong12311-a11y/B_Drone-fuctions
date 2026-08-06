cmake_minimum_required(VERSION 3.0.2)
project(rgb_qr_detect)

# 设置 C++14 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找 catkin 和相关宏包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

## 查找 OpenCV
find_package(OpenCV REQUIRED)

## 声明 catkin 包及其依赖
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

## 指定头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## 编译可执行文件 (已更改为 detect)
add_executable(detect src/detect.cpp)
target_link_libraries(detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## 安装可执行文件
install(TARGETS detect
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## 安装 launch 文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)