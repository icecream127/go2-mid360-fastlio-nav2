# Go2 + MID-360：Gazebo 三维建图、重定位与导航

本项目基于 WSL2、ROS 2 Humble、Gazebo Classic 和 CHAMP 四足控制器，为 Unitree Go2 仿真接入 MID-360 三维激光雷达，并实现 FAST-LIO 三维建图、基于 PCD 地图的 ICP 重定位和 Nav2 导航。

## 项目功能

- 在 Gazebo Classic 11 中仿真 Go2 与 MID-360；
- 发布 `/livox/lidar` 三维 PointCloud2、`/livox/lidar_custom` Livox CustomMsg，以及用于 Nav2 的二维 `/scan`；
- 使用 FAST-LIO 完成三维激光惯性里程计和 PCD 建图；
- 使用 ICP 在已有 PCD 三维地图中进行重定位；
- 使用 Nav2 完成路径规划、目标点导航和二维避障。

导航使用的 TF 坐标链如下：

```text
map -> odom -> base_footprint -> base_link -> mid360_link
```

其中 `/odom` 由 FAST-LIO 的 `/Odometry` 转换而来。
主导航流程不使用 Gazebo 的 `/odom/ground_truth` 真值里程计。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- 一个 ROS 2 工作空间，例如 `~/go2_ws`

## 安装

将本项目克隆到工作空间的 `src` 目录后，执行一次依赖安装脚本：

```bash
cd ~/go2_ws/src/unitree-go2-ros2
chmod +x tools/setup_mid360_dependencies.bash
./tools/setup_mid360_dependencies.bash
```

该脚本会安装 ROS 依赖，下载指定版本的 Livox SDK2、`livox_ros_driver2`、`livox_laser_simulation_ros2` 与 `FAST_LIO_ROS2`，自动应用本项目所需补丁，并编译整个工作空间。

## 使用流程

### 1. 三维建图

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash gui:=true show_rviz:=true
```

另开终端后可用键盘控制机器人运动：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

建图结束时，在第一个终端按 `Ctrl+C`。配置的三维地图输出路径为：

```text
~/go2_ws/maps/mid360_3d.pcd
```

### 2. 从 PCD 生成 Nav2 二维地图

Nav2 使用从三维点云指定高度范围投影得到的二维栅格地图：

```bash
python3 ~/go2_ws/src/unitree-go2-ros2/robots/configs/go2_config/scripts/pcd_to_nav2_map.py \
  ~/go2_ws/maps/mid360_3d.pcd ~/go2_ws/maps/mid360_3d_nav \
  --resolution 0.05 --z-min 0.35 --z-max 1.50 --padding 0.30 \
  --min-points 2 --dilation-cells 1
```

命令会生成相匹配的：

```text
mid360_3d_nav.pgm
mid360_3d_nav.yaml
```

### 3. 已有地图上的重定位与导航

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash gui:=true show_rviz:=true
```

等待系统加载完成后，在 RViz 中：

1. 使用 **2D Pose Estimate** 给机器人一个大概的初始位置与朝向；
2. 等待 ICP 点云匹配稳定；
3. 使用 **Nav2 Goal** 设置导航目标点。

若需换用另一张地图，提供同一场景对应的 PCD 与二维投影地图：

```bash
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash \
  gui:=true show_rviz:=true \
  pcd_map:=~/go2_ws/maps/example.pcd \
  nav_map:=~/go2_ws/maps/example_nav.yaml
```

## 已知限制

- 当前使用的是 Gazebo Classic 射线模型模拟的 MID-360 风格点云，并非真实 MID-360 硬件驱动，也不是硬件级精度的传感器仿真；
- ICP 需要相对合理的人工初始位姿；在高度对称、特征不足或地图变化很大的场景中可能匹配失败；
- Nav2 使用 PCD 的二维高度切片进行规划和避障，FAST-LIO 与 ICP 仍是三维定位来源。

## 可复现性说明

`patches/` 保存了本项目对指定版本 Livox Gazebo 插件和 FAST-LIO 的必要补丁：前者为 FAST-LIO 补充 Livox CustomMsg 输出，后者保证在 `Ctrl+C` 结束建图时保存完整 PCD 地图。

本项目建立在 Unitree Go2 description、CHAMP、Livox SDK2、`livox_ros_driver2`、LCAS `livox_laser_simulation_ros2` 与 `FAST_LIO_ROS2` 等开源项目之上，相关源码分别遵循其原始许可证。
