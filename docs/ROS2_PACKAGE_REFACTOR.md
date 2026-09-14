# ROS 2 功能包重构（2026-09-14）

## 代码归属

| 新包 | 维护的功能 |
| --- | --- |
| go2_mid360_sim | simulation.launch.py、MID-360 点云转扫描参数、传感器 RViz、仿真世界 |
| go2_fastlio_localization | mapping/localization 启动、FAST-LIO 参数、ICP、里程计桥接、PCD 地图转换 |
| go2_nav_bringup | navigation 总启动、mapping/localization 快捷入口、Nav2 参数、导航 RViz、参考地图 |

go2_config 保留步态、关节、控制参数以及历史实验。四个旧主启动文件转发到新包；
三个旧 Python 可执行文件转发到新 Python 模块。旧安装地图路径通过 CMake 安装
go2_nav_bringup 中的参考地图提供兼容，仓库只保存一份参考地图。
原机器人 URDF/Xacro 仍由 go2_description 管理；CHAMP 原始作者信息保持不变。

## 标准入口

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_config go2_mid360_sim go2_fastlio_localization go2_nav_bringup
source install/setup.bash
ros2 launch go2_nav_bringup navigation.launch.py gui:=true show_rviz:=true
```

单独的标准命令：

```bash
ros2 launch go2_nav_bringup mapping.launch.py
ros2 launch go2_nav_bringup localization.launch.py
ros2 launch go2_mid360_sim simulation.launch.py
ros2 run go2_fastlio_localization pcd_to_nav2_map --help
```

tools/run_*.bash 主流程已切换到新包。依赖安装脚本从 go2_nav_bringup/maps
安装地图，Dockerfile 调用的也是此脚本，已有镜像需要重新构建才能包含新结构。

## 本次验证

- 四个相关包 colcon build 成功，Python 三个入口均由 ROS 2 发现。
- 新 navigation 和旧 fast_lio_nav2_demo 启动参数解析成功。
- 实际启动 Gazebo、Livox 点云仿真、FAST-LIO 和新包 ICP 节点。
- 自动初始位姿生效，ICP 持续匹配并建立 map -> odom。
- 本机普通通信域首次服务发现不完整；使用 ROS_DOMAIN_ID=67、
  ROS_LOCALHOST_ONLY=1 的独立通信域复测，Nav2 管理节点激活。
- 对 /navigate_to_pose 发送 map 坐标 (1.0, 0.0)，收到 SUCCEEDED。
- 参考 PCD 与现有工作空间地图 SHA-256 一致，原地图未重新生成。
- 测试仿真已停止。

这次没有重新验证 GUI 显示、完整建图保存及 Docker 运行。
当前 WSL 中 Docker 命令不可用，Docker 重建仍需恢复 Desktop 的 WSL 集成后执行。
独立通信域仅用于此次测试，未作为项目默认值写入。
