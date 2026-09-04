# Unitree Go2 + Livox MID-360 仿真说明

本文记录在 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11 环境中，将
Unitree Go2 原有 Velodyne/激光雷达配置替换为 Livox MID-360 的过程及使用方法。

## 1. 当前完成状态

目前已经完成：

- Go2 在 Gazebo 中正常生成；
- MID-360 Xacro 模型已挂载到 Go2；
- Livox Gazebo 点云插件正常加载；
- `/livox/lidar` 正常发布 `sensor_msgs/msg/PointCloud2`；
- 点云坐标系为 `mid360_link`；
- TF 链 `odom -> base_link -> mid360_link` 正常；
- RViz 自动加载 MID-360 点云显示配置；
- Python 验证脚本检查通过。

当前测试结果：

| 项目 | 结果 |
| --- | --- |
| 点云话题 | `/livox/lidar` |
| 点云坐标系 | `mid360_link` |
| 单帧点数 | 40000 |
| 实测频率 | 约 7 Hz |
| 点云字段 | `x, y, z, intensity, tag, line` |
| RViz Fixed Frame | `odom` |

尚未完成：

- FAST-LIO；
- SLAM 建图；
- Nav2 导航；
- 真机 MID-360 驱动及网络配置。

## 2. 环境

- Ubuntu 22.04（WSL 2）
- ROS 2 Humble
- Gazebo Classic 11.10
- RViz 2
- CHAMP 四足控制器

工作空间：

```text
~/go2_ws
```

Go2 项目：

```text
~/go2_ws/src/unitree-go2-ros2
```

## 3. 新增依赖

工作空间中新增：

```text
~/go2_ws/src/Livox-SDK2
~/go2_ws/src/livox_ros_driver2
~/go2_ws/src/livox_laser_simulation_ros2
```

Livox SDK2 使用无 root 的方式安装到：

```text
~/go2_ws/install/livox-sdk2
```

每个新终端在运行仿真前需要设置：

```bash
export LD_LIBRARY_PATH=~/go2_ws/install/livox-sdk2/lib:$LD_LIBRARY_PATH
```

## 4. 对原项目的修改

### 4.1 Go2 Xacro

修改文件：

```text
robots/descriptions/go2_description/xacro/robot_VLP.xacro
```

主要修改：

- 引入 `ros2_livox_simulation/urdf/mid360.xacro`；
- 移除原 Velodyne/2D 激光雷达引用；
- 新增 `mid360_link`；
- 点云话题设置为 `/livox/lidar`。

MID-360 安装参数：

```xml
<xacro:mid360
    name="mid360_link"
    parent="base_link"
    topic="/livox/lidar">
    <origin xyz="0.2 0 0.1177" rpy="0 0 0"/>
</xacro:mid360>
```

### 4.2 Gazebo 启动文件

新增：

```text
robots/configs/go2_config/launch/gazebo_mid360.launch.py
```

该启动文件负责启动：

- Go2/CHAMP 控制节点；
- Gazebo 服务端和客户端；
- MID-360 仿真插件；
- 可选的 RViz。

### 4.3 RViz 配置

新增：

```text
robots/configs/go2_config/config/mid360.rviz
```

配置内容：

- Fixed Frame 为 `odom`；
- PointCloud2 默认启用；
- Topic 为 `/livox/lidar`；
- Position Transformer 为 `XYZ`；
- Color Transformer 为 `Intensity`；
- 旧的 Hokuyo LaserScan 默认关闭。

### 4.4 验证脚本

新增：

```text
tools/check_mid360.py
```

脚本用于检查点云话题、坐标系、点数、字段和发布频率。

## 5. 编译

修改源码或配置后，在工作空间执行：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=~/go2_ws/install/livox-sdk2/lib:$LD_LIBRARY_PATH

colcon build --packages-select \
  livox_ros_driver2 \
  ros2_livox_simulation \
  go2_description \
  go2_config

source install/setup.bash
```

如果只修改了启动文件或 RViz 配置，可以只构建：

```bash
colcon build --packages-select go2_config
source install/setup.bash
```

## 6. 启动仿真

### 6.1 Gazebo + RViz

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=~/go2_ws/install/livox-sdk2/lib:$LD_LIBRARY_PATH

ros2 launch go2_config gazebo_mid360.launch.py rviz:=true
```

### 6.2 只启动 Gazebo

```bash
ros2 launch go2_config gazebo_mid360.launch.py rviz:=false
```

RViz 中应看到：

- Go2 机器人模型；
- `/livox/lidar` 点云；
- Fixed Frame 为 `odom`。

## 7. 命令行验证

查看点云话题：

```bash
ros2 topic info /livox/lidar
```

查看发布频率：

```bash
ros2 topic hz /livox/lidar
```

查看单帧点数：

```bash
ros2 topic echo /livox/lidar --once --field width
```

查看 TF：

```bash
ros2 run tf2_ros tf2_echo odom mid360_link
```

刚启动时可能短暂出现：

```text
Invalid frame ID "odom"
```

如果随后能够持续输出坐标变换，说明 TF 正常，只是 `odom` 发布节点启动稍慢。

## 8. Python 验证

先在一个终端启动仿真，再在另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash

python3 \
  ~/go2_ws/src/unitree-go2-ros2/tools/check_mid360.py
```

正常输出示例：

```text
Topic       : /livox/lidar
Frame       : mid360_link
Messages    : 10
Points/frame: 40000
Rate        : 7.06 Hz
Fields      : intensity, line, tag, x, y, z
[PASS] messages received
[PASS] non-empty cloud
[PASS] frame_id is mid360_link
[PASS] XYZ fields present
```

## 9. RViz 注意事项

推荐设置：

```text
Global Options -> Fixed Frame -> odom
PointCloud2 -> Topic -> /livox/lidar
```

不要长期使用 `base_link` 作为 Fixed Frame。Go2 的 `base_link` 位于机身内部，
RViz 网格会穿过机身，看起来像机器人一半位于地面以下。这只是显示参考系不同，
不会影响 Gazebo 中的物理模型。

## 10. 常见问题

### 10.1 Gazebo 进程运行但没有窗口

如果连单独运行 `gazebo` 都无法显示窗口，通常是 WSLg 会话异常。先在 Windows
PowerShell 中执行：

```powershell
wsl --shutdown
```

然后重新打开 Ubuntu。

### 10.2 Gazebo 在线模型库警告

可以禁用在线模型库：

```bash
export GAZEBO_MODEL_DATABASE_URI=""
```

类似以下警告不会阻止本地仿真：

```text
Unable to connect to model database
Missing model.config for model ".../.git"
```

### 10.3 `hold_joints` 重复声明

日志中可能出现：

```text
Parameter 'hold_joints' has already been declared
```

这是原 Go2 `gazebo_ros2_control` 配置的重复参数提示。当前控制器仍能进入 active
状态，不影响 MID-360 点云发布。

### 10.4 仿真较慢

当前使用轻量解析式三维扫描器，默认生成 `180 x 12` 条三维射线（5 Hz），并额外
保留 360 点水平 `/scan`。同时显示 Gazebo 和 RViz 仍会降低 WSL 仿真实时率；建图时
建议 Gazebo 使用 `gui:=false`，只显示 RViz。

## 11. 备份

之前测试过但未采用的 MID-360 仓库保留在：

```text
~/mid360_simulation_legacy
```

Gazebo 旧窗口配置备份在：

```text
~/.gazebo/gui.ini.codex-backup
```

## 12. FAST-LIO 三维建图（当前方案）

当前 `/livox/lidar` 已不是平面伪点云。三维扫描器会发布：

- `/livox/lidar`：带 XYZ、强度、逐点时间和 ring 的 `PointCloud2`；
- `/livox/lidar_custom`：带 `offset_time` 的 Livox `CustomMsg`，供 FAST-LIO；
- `/scan`：水平二维切片，继续供 AMCL/Nav2 使用。

启动三维建图：

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash
```

默认不显示 Gazebo，只打开 FAST-LIO RViz。需要同时显示 Gazebo 时：

```bash
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash gui:=true
```

自动执行一段按仿真时间计算的采集路线：

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
ros2 run go2_config drive_3d_mapping_route.py --ros-args -p use_sim_time:=true
```

验证三维输入、逐点时间戳和 FAST-LIO 输出：

```bash
ros2 run go2_config check_mid360_3d.py
```

保存三维 PCD：

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

默认保存到：

```text
~/go2_ws/maps/mid360_3d.pcd
```

不启动仿真，直接查看已经保存的三维地图：

```bash
./src/unitree-go2-ros2/tools/view_mid360_3d_map.bash
```

三维建图和二维导航可以并存。FAST-LIO 使用完整三维点云和 IMU 建立 PCD 地图；
AMCL/Nav2 继续使用 `/scan` 和二维 occupancy map。现有 AMCL 导航入口仍为：

```bash
./src/unitree-go2-ros2/tools/run_amcl_nav2_demo.bash
```
