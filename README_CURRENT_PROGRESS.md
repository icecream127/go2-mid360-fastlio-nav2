# Unitree Go2 + MID-360 项目当前进度

> 更新时间：2026-08-02  
> 工作空间：`~/go2_ws`  
> 主项目：`~/go2_ws/src/unitree-go2-ros2`

本文记录截至目前在原始 `unitree-go2-ros2` 项目上完成的工作、产生或更新的文件、验证结果、使用命令和后续任务。

## 1. 当前结论

目前已经完成以下主线：

1. 将 Go2 原有雷达配置替换为 MID-360；
2. 在 Gazebo Classic 中生成 MID-360 三维点云和二维扫描；
3. 保留 `/scan`，用于 SLAM Toolbox、AMCL 和 Nav2；
4. 接入 FAST-LIO ROS 2，使用 MID-360 点云和 IMU完成三维建图；
5. 新建带障碍物的仿真世界，并自动控制机器狗遍历场景；
6. 保存并验证三维 PCD 地图；
7. 保留二维 AMCL/Nav2 演示链路。

当前所处阶段是：**三维建图已经完成，下一步是已有三维地图上的全局重定位，然后把定位结果接入 Nav2。**

当前还没有完成：

- 重新启动后在 `mid360_3d.pcd` 中确定机器人全局位置；
- 由三维点云匹配稳定发布 `map -> odom`；
- 三维重定位与 Nav2 的最终端到端联调；
- 真机 MID-360 驱动、网络和外参标定。

## 2. 已验证的功能

### 2.1 Go2 和 Gazebo

- Go2 能够在 Gazebo Classic 11 中生成；
- CHAMP 控制器可以接收速度指令并让机器狗移动；
- Gazebo GUI 可以通过启动参数选择显示或关闭；
- 关闭了默认高 CPU 的足端接触辅助节点；
- 调整了仿真世界物理参数，降低加载和运行压力；
- 增加了适合建图和导航测试的墙体、柱体等障碍物。

### 2.2 MID-360 仿真

MID-360 安装在 `base_link` 前上方：

```text
xyz = 0.2 0 0.1177
rpy = 0 0 0
```

当前数据接口：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | RViz、普通点云工具和检查 |
| `/livox/lidar_custom` | `livox_ros_driver2/msg/CustomMsg` | FAST-LIO 输入 |
| `/scan` | `sensor_msgs/msg/LaserScan` | SLAM Toolbox、AMCL、Nav2 避障 |
| `/imu/data` | IMU | FAST-LIO 输入 |
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | FAST-LIO 配准后的三维点云 |

为了避免 Gazebo Classic 的三维 Multiray 传感器导致加载缓慢，新增了轻量级 Python MID-360 仿真节点。它读取静态 SDF 世界，对地面、Box 和 Cylinder 障碍物做射线求交，并同时发布：

- 180 × 12 三维射线；
- 约 5 Hz 的三维点云；
- 带非零 `offset_time` 的 Livox `CustomMsg`；
- 360 点水平 `/scan`。

### 2.3 二维建图和导航

- 已将 MID-360 水平切片转换/发布为 `/scan`；
- `/scan` 已验证存在有效发布者，实测约 6～7 Hz；
- 已调整 SLAM Toolbox 参数以适配 MID-360 的 15 m 量程；
- 已创建 SLAM、Nav2 Goal 和 AMCL 启动文件；
- 已生成一份二维地图：`~/go2_ws/maps/mid360_mapping_initial.pgm`；
- 已保留用于 AMCL/Nav2 的二维地图与测试脚本。

这里的二维链路不会代替三维雷达：最终计划是用三维点云做全局重定位，用二维地图和 `/scan` 让 Nav2 规划及避障。

### 2.4 FAST-LIO 三维建图

- 已将 `FAST_LIO_ROS2` 加入工作空间并完成编译；
- FAST-LIO 能读取 `/livox/lidar_custom` 和 `/imu/data`；
- IMU 初始化、KD-Tree 初始化和点云配准均已成功；
- `/cloud_registered` 有有效点云并存在明显的 Z 轴跨度；
- `/map_save` 服务可以保存 PCD；
- 已创建两段自动建图路线，覆盖世界西侧、中心区域和东侧。

最终三维地图：

```text
~/go2_ws/maps/mid360_3d.pcd
```

验证结果：

| 项目 | 结果 |
| --- | --- |
| 文件大小 | 1,203,417 bytes（约 1.2 MB） |
| 点数 | 37,599 |
| X 范围 | -7.119 ～ 6.890 m |
| Y 范围 | -6.055 ～ 5.965 m |
| Z 范围 | -0.686 ～ 1.605 m |

自动建图前的地图已备份为：

```text
~/go2_ws/maps/mid360_3d_before_auto.pcd
```

## 3. 新增的第三方源码和依赖

工作空间 `~/go2_ws/src` 中新增或使用了：

| 路径 | 用途 |
| --- | --- |
| `Livox-SDK2` | Livox SDK |
| `livox_ros_driver2` | Livox ROS 2 消息和驱动接口 |
| `livox_laser_simulation_ros2` | MID-360 模型及早期 Gazebo 插件接入 |
| `FAST_LIO_ROS2` | 三维激光惯性里程计和建图 |
| `go2_navigation` | 导航相关实验包 |

Livox SDK 的本地库路径为：

```text
~/go2_ws/install/livox-sdk2/lib
```

项目提供的 Bash 包装脚本会自动设置该路径，正常使用时不必每个终端手动执行 `export LD_LIBRARY_PATH=...`。

## 4. 原项目中修改的文件

以下是 Git 当前能够识别的已修改文件。

### `robots/descriptions/go2_description/xacro/robot_VLP.xacro`

- 引入 `mid360.xacro`；
- 移除原 Velodyne 引用；
- 在 `base_link` 上安装 `mid360_link`；
- 点云话题设置为 `/livox/lidar`。

### `robots/descriptions/go2_description/launch/description.launch.py`

- 使用 `ParameterValue(..., value_type=str)` 传递机器人描述；
- 避免 Xacro/XML 字符串被错误推断成其他参数类型。

### `champ/champ_gazebo/launch/gazebo.launch.py`

- Gazebo 客户端改为由 `gui` 参数控制；
- 增加 `contact_sensor` 参数，默认不启动高 CPU 足端接触节点；
- 对 `robot_description` 使用明确的字符串参数类型。

### `champ/champ_bringup/launch/bringup.launch.py`

- `footprint_to_odom_ekf` 是否启动改由 `publish_odom_tf` 控制；
- 避免地面真值里程计、CHAMP 里程计和 FAST-LIO 同时发布冲突 TF。

### `robots/configs/go2_config/config/autonomy/slam.yaml`

- 关闭大量调试日志；
- 最大雷达距离由 5 m 改为 15 m；
- 加快地图更新；
- 降低生成新扫描节点所需的移动距离和转角；
- 增加 TF 缓冲和超时时间。

### `robots/configs/go2_config/config/autonomy/navigation.yaml`

- AMCL 最大激光距离改为 15 m；
- 初始朝向改为 0 rad。

### `robots/configs/go2_config/CMakeLists.txt`

- 安装新增的 MID-360、地面真值里程计、检查和自动路线 Python 程序。

### `robots/configs/go2_config/package.xml`

- 增加 `pointcloud_to_laserscan`、`rclpy`、`nav_msgs`、`sensor_msgs`、`livox_ros_driver2`、`geometry_msgs` 和 `tf2_ros` 运行依赖。

## 5. 新增的项目文件

### 5.1 启动文件

| 文件 | 用途 |
| --- | --- |
| `robots/configs/go2_config/launch/gazebo_mid360.launch.py` | 启动 Go2、Gazebo、MID-360、可选 RViz 和里程计 |
| `robots/configs/go2_config/launch/slam_mid360.launch.py` | 使用 `/scan` 启动二维 SLAM |
| `robots/configs/go2_config/launch/nav2_goal_demo.launch.py` | 启动 Nav2 Goal 演示 |
| `robots/configs/go2_config/launch/amcl_nav2_demo.launch.py` | 加载已有二维地图并启动 AMCL + Nav2 |
| `robots/configs/go2_config/launch/fast_lio_3d_mapping.launch.py` | 启动仿真、FAST-LIO 和可选 RViz |
| `robots/configs/go2_config/launch/view_3d_map.launch.py` | 不启动 Gazebo，单独发布和查看 PCD |

### 5.2 配置和世界

| 文件 | 用途 |
| --- | --- |
| `robots/configs/go2_config/config/mid360.rviz` | MID-360 点云 RViz 配置 |
| `robots/configs/go2_config/config/mid360_to_scan.yaml` | 点云到水平扫描相关参数 |
| `robots/configs/go2_config/config/fast_lio_mid360_sim.yaml` | FAST-LIO 的 MID-360、IMU、外参和 PCD 保存配置 |
| `robots/configs/go2_config/config/fastdds_udp.xml` | 限制 DDS 使用 UDP，改善 WSL 仿真通信稳定性 |
| `robots/configs/go2_config/worlds/mid360_mapping.world` | 建图/导航测试世界和物理参数 |

FAST-LIO 当前关键配置：

```text
lidar topic: /livox/lidar_custom
imu topic:   /imu/data
range:       15 m
scan rate:   5 Hz
extrinsic T: [0.2, 0.0, 0.1177]
map output:  /home/ice/go2_ws/maps/mid360_3d.pcd
```

### 5.3 Python 节点

| 文件 | 用途 |
| --- | --- |
| `robots/configs/go2_config/scripts/fast_mid360_3d.py` | 轻量级三维 MID-360 射线仿真，同时发布 PointCloud2、CustomMsg 和 LaserScan |
| `robots/configs/go2_config/scripts/fast_mid360_scan.py` | 早期快速二维扫描实现 |
| `robots/configs/go2_config/scripts/ground_truth_odom.py` | 发布平面地面真值里程计和相应 TF |
| `robots/configs/go2_config/scripts/check_mid360_3d.py` | 检查原始点云、Livox 时间偏移和 FAST-LIO 注册点云 |
| `robots/configs/go2_config/scripts/drive_3d_mapping_route.py` | 自动遍历主要建图路线 |
| `robots/configs/go2_config/scripts/drive_3d_mapping_east.py` | 补充扫描世界东侧区域 |
| `tools/check_mid360.py` | 检查基础 MID-360 点云话题、字段、帧和频率 |

### 5.4 操作脚本

| 文件 | 用途 |
| --- | --- |
| `tools/run_mid360_sim.bash` | 一键启动 MID-360 仿真 |
| `tools/run_nav2_goal_demo.bash` | 一键启动 Nav2 Goal 演示 |
| `tools/run_amcl_nav2_demo.bash` | 一键启动二维地图 AMCL + Nav2 |
| `tools/test_amcl_nav_goal.bash` | 自动发送导航目标进行测试 |
| `tools/run_fast_lio_3d_mapping.bash` | 一键启动 FAST-LIO 三维建图 |
| `tools/view_mid360_3d_map.bash` | 不启动仿真，单独查看已保存 PCD |

### 5.5 说明文件

- `README_MID360.md`：早期 MID-360 接入记录，其中部分“未完成”状态已经过时；
- `README_CURRENT_PROGRESS.md`：本文件，以本文件记录的当前进度为准。

`__pycache__` 和 `.pyc` 文件是 Python/ROS 运行时自动生成的缓存，不属于功能源码，后续应加入 `.gitignore` 或清理后再提交。

## 6. 地图文件

地图位于工作空间级目录，而不是 Git 仓库内部：

| 文件 | 内容 |
| --- | --- |
| `~/go2_ws/maps/mid360_mapping_initial.pgm` | 二维占据栅格图像 |
| `~/go2_ws/maps/mid360_mapping_initial.yaml` | 二维地图元数据 |
| `~/go2_ws/maps/mid360_3d.pcd` | 当前最终三维点云地图 |
| `~/go2_ws/maps/mid360_3d_before_auto.pcd` | 自动补图前的三维地图备份 |

项目包内部还保留了原地图和测试地图：

```text
robots/configs/go2_config/maps/map.pgm
robots/configs/go2_config/maps/map.yaml
robots/configs/go2_config/maps/playground.pgm
robots/configs/go2_config/maps/playground.yaml
```

## 7. 常用命令

### 7.1 编译当前相关包

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=~/go2_ws/install/livox-sdk2/lib:${LD_LIBRARY_PATH:-}

colcon build --packages-select \
  livox_ros_driver2 \
  ros2_livox_simulation \
  go2_description \
  go2_config \
  fast_lio

source install/setup.bash
```

### 7.2 启动普通 MID-360 仿真

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_mid360_sim.bash
```

### 7.3 启动 FAST-LIO 三维建图

显示 Gazebo 和 RViz：

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash \
  gui:=true show_rviz:=true
```

无界面运行：

```bash
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash \
  gui:=false show_rviz:=false
```

### 7.4 验证三维链路

仿真和 FAST-LIO 已启动时，在另一终端执行：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run go2_config check_mid360_3d.py
```

检查内容包括：

- `/livox/lidar` 是否包含有效 XYZ 和 Z 轴跨度；
- `/livox/lidar_custom` 是否有有效点数及非零点时间偏移；
- `/cloud_registered` 是否由 FAST-LIO 正常发布。

### 7.5 保存 FAST-LIO 地图

```bash
ros2 service call /map_save std_srvs/srv/Trigger
```

保存位置由 `fast_lio_mid360_sim.yaml` 中的 `map_file_path` 决定。

### 7.6 单独查看最终三维地图

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/view_mid360_3d_map.bash
```

此命令不启动 Gazebo，只启动 PCD 发布节点、静态 TF 和 RViz。

## 8. 坐标系和 RViz 注意事项

- 普通 Gazebo/MID-360 仿真中，RViz 的 Fixed Frame 使用 `odom`；
- FAST-LIO 建图及 PCD 地图使用 `camera_init` 作为全局帧；
- 将普通仿真的 Fixed Frame 改成 `base_link` 后，网格会跟随机器人机身，因此机器人看起来可能有一部分位于网格下方；这不表示 Gazebo 中机器人穿过地面；
- 三维重定位接入 Nav2 时，需要统一成标准的 `map -> odom -> base_link -> mid360_link` TF 链；
- 不应同时让多个节点发布相同的 `odom -> base_link` TF。

## 9. 当前架构

三维建图链路：

```text
Gazebo 中的 Go2 位姿
        |
        v
fast_mid360_3d.py
        |-- /livox/lidar --------> RViz/检查工具
        |-- /livox/lidar_custom -> FAST-LIO
        `-- /scan ---------------> 二维 SLAM/Nav2

/imu/data -----------------------> FAST-LIO
FAST-LIO ------------------------> /cloud_registered + 三维 PCD
```

已经联通的最终导航链路：

```text
MID-360 实时三维点云 + IMU
        |
        v
已有 PCD 地图上的三维全局重定位
        |
        `----> map -> odom

二维地图 + /scan + TF
        |
        v
Nav2 路径规划和实时避障
        |
        v
/cmd_vel -> CHAMP -> Go2 行走
```

## 10. 三维重定位与 Nav2 联调结果（2026-08-03）

本阶段已经完成：

- 使用 `pcd_icp_localizer.py` 将 FAST-LIO 实时注册点云匹配到
  `~/go2_ws/maps/mid360_3d.pcd`；
- TF 所有权已经固定为 ICP 节点发布 `map -> odom`，仿真里程计发布
  `odom -> base_footprint`，机器人模型发布 `base_footprint -> base_link -> ...`；
- Nav2 不启动 AMCL，避免两个定位器同时发布 `map -> odom`；
- 三维 PCD 已投影成 Nav2 地图 `mid360_3d_nav.yaml/.pgm`；
- Nav2 的局部与全局代价地图只使用 MID-360 生成的 `/scan`，不再等待不存在的
  `/base/scan`、RealSense 或 ZED；
- 所有 Nav2 节点统一使用仿真时间，并将规划基座设为 `base_footprint`；
- 已实测发送 `(0.8, 0.0)` 目标，机器人从约 `(0.02, 0.00)` 行走到
  `(0.55, 0.01)`，Nav2 返回 `SUCCEEDED`；
- 运动期间 ICP RMSE 约为 `0.069~0.075 m`，停止后回到约 `0.060~0.062 m`，
  定位没有丢失。

本阶段新增的主要文件：

- `robots/configs/go2_config/scripts/pcd_to_nav2_map.py`
- `robots/configs/go2_config/maps/mid360_3d_nav.pgm`
- `robots/configs/go2_config/maps/mid360_3d_nav.yaml`
- `robots/configs/go2_config/config/autonomy/fast_lio_nav2.yaml`
- `robots/configs/go2_config/launch/fast_lio_nav2_demo.launch.py`
- `tools/run_fast_lio_nav2_demo.bash`

### 10.1 一条命令启动最终演示

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash \
  gui:=true show_rviz:=true
```

等待 Gazebo、FAST-LIO 和 Nav2 加载。RViz 的 Fixed Frame 应为 `map`。

### 10.2 设置初始位姿

在 RViz 点击 `2D Pose Estimate`，在机器狗实际位置附近拖出朝向。当前仿真从原点
启动，因此可在 `(0, 0)` 附近设置；也可以在另一终端执行：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
```

终端出现 `ICP accepted` 和 `Nav2 TF bridge ready: map -> odom` 后，重定位完成。

### 10.3 发送导航目标

在 RViz 点击 `Nav2 Goal`（某些 RViz 版本显示为 `2D Goal Pose`），在自由区域设置
目标即可。命令行测试目标如下：

```bash
ros2 action send_goal /navigate_to_pose \
  nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.8, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback
```

### 10.4 重新生成二维规划地图

如果以后重新建立了三维 PCD，执行：

```bash
cd ~/go2_ws/src/unitree-go2-ros2
python3 robots/configs/go2_config/scripts/pcd_to_nav2_map.py \
  ~/go2_ws/maps/mid360_3d.pcd \
  robots/configs/go2_config/maps/mid360_3d_nav

cd ~/go2_ws
colcon build --packages-select go2_config
```

## 11. 下一步优化（不影响当前演示）

1. 将 `xy_goal_tolerance` 从当前 `0.25 m` 调小到 `0.15 m`，提高停车精度；
2. 调整 DWB 和速度平滑参数，使四足步态转弯更连贯；
3. 在更多初始位置、遮挡和长距离路径上测试 ICP 全局收敛范围；
4. 需要实机部署时，用真实 Livox 驱动和真实里程计替换仿真传感器与真值里程计。

## 12. Git 状态提醒

当前修改和新增文件尚未形成正式 Git 提交。继续开发前建议：

1. 删除或忽略 `__pycache__/`、`*.pyc`；
2. 核对无关改动；
3. 将本阶段作为一个独立提交保存；
4. 再开始三维重定位阶段，便于随时回退。
