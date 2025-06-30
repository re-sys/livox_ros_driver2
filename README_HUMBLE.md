# Livox ROS Driver2 - Humble Version

这是Livox ROS Driver2的纯Humble版本，专门为ROS2 Humble设计，可以直接使用`colcon build`进行编译。

## 编译

在workspace根目录下直接运行：

```bash
colcon build --packages-select livox_ros_driver2
```

或者编译整个workspace：

```bash
colcon build
```

## 依赖项

确保已安装以下依赖项：

- ROS2 Humble
- PCL (Point Cloud Library)
- APR (Apache Portable Runtime)
- Livox SDK

安装Livox SDK：
```bash
# 下载并安装Livox SDK
# 参考：https://github.com/Livox-SDK/Livox-SDK2
```

## 使用方法

1. 编译完成后，source环境：
```bash
source install/setup.bash
```

2. 运行驱动节点：
```bash
# 对于MID360激光雷达
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 对于HAP激光雷达
ros2 launch livox_ros_driver2 msg_HAP_launch.py

# 带RViz的可视化启动
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
```

## 配置

配置文件位于`config/`目录下，可以根据您的激光雷达型号进行相应配置。

## 发布的话题

- `/livox/lidar`: 激光雷达点云数据
- `/livox/imu`: IMU数据（如果支持）

## 注意事项

- 此版本已移除ROS1相关代码，只支持ROS2 Humble
- 不再需要`build.sh`脚本，直接使用`colcon build`即可
- 确保Livox SDK已正确安装并配置 