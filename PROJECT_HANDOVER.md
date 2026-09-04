# Go2 + MID360 + FAST-LIO + ICP + Nav2 项目交接清单

更新时间：2026-08-31  
当前工作区：`/home/ice/go2_ws`  
主仓库：`/home/ice/go2_ws/src/unitree-go2-ros2`

## 0. 先给结论

当前工程是一个 **Go2 Gazebo 仿真中，使用简化合成三维点云验证 FAST-LIO、ICP 重定位和 Nav2 平面导航链路** 的原型。

它能够启动 Gazebo、显示 RViz、发布点云、建立 PCD、运行 ICP，并让 Nav2 Goal 驱动机器狗行走；但它**不是**已完成真实 MID360 雷达高保真仿真、真实激光里程计闭环导航或真机可用系统。

以下两点必须在任何交接、报告或演示中明确：

1. 当前 MID360 数据来自自写几何射线脚本，不是官方/真实 MID360 传感器模型。
2. 当前导航所用 `/odom` 来自 Gazebo ground truth；不能把 Nav2 行走成功表述为“不依赖真值的激光自主导航”。

## 1. 当前可运行的演示

启动已有三维地图上的 ICP 重定位和 Nav2：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash \
  gui:=true show_rviz:=true
```

启动后顺序大致为：Gazebo/控制器 → FAST-LIO → ICP 定位器 → Nav2 → RViz。RViz 被故意延迟约 22 秒启动。

流程：在 RViz 使用 `2D Pose Estimate` 给出大概初始位姿；终端出现 `ICP accepted` 和 `Nav2 TF bridge ready: map -> odom` 后，使用 `Nav2 Goal` 发目标点。

三维建图命令：

```bash
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash \
  gui:=true show_rviz:=true
```

已存在的主要地图文件：

- `robots/configs/go2_config/maps/mid360_3d.pcd`：三维 PCD；
- `robots/configs/go2_config/maps/mid360_3d_nav.yaml/.pgm`：给 Nav2 使用的二维栅格投影图；
- 工作区下还有 `~/go2_ws/maps/` 的副本。

## 2. 实际架构，而非宣传口径

```text
mid360_mapping.world 的静态碰撞几何体
        ↓
fast_mid360_3d.py（解析几何 + 射线求交）
        ├─ /livox/lidar         PointCloud2
        ├─ /livox/lidar_custom  Livox CustomMsg
        └─ /scan                完美的水平 LaserScan
        ↓
FAST-LIO
        ├─ /Odometry
        ├─ /cloud_registered
        └─ camera_init → body
        ↓
pcd_icp_localizer.py（PCD ICP）
        ├─ map → camera_init（3D）
        └─ map → odom（仅 x/y/yaw，给 Nav2）
        ↓
Nav2 的 /cmd_vel
        ↓
CHAMP 四足控制器 → ros2_control → Gazebo 关节
```

## 3. 最高优先级：会造成“结果看起来比真实更好”的部分

### P0-1：MID360 不是高保真或官方 Gazebo 雷达仿真

相关文件：

```text
robots/configs/go2_config/scripts/fast_mid360_3d.py
robots/configs/go2_config/worlds/mid360_mapping.world
```

该脚本读取 world 中**静态**模型的 `box`、`cylinder` 和 `plane` collision，再调用 `_ray_box`、`_ray_cylinder`、`_ray_plane` 计算交点。它以固定 `180 × 12 = 2160` 条射线、`5 Hz` 发布点云；点的 intensity 固定为 `100`，Livox 时间戳和 line 信息为人工构造。

它模拟了基本的量程、遮挡、雷达随机器人移动和简单三维几何距离；它**没有**模拟真实 MID360 的扫描花样、真实点频与时序、反射率、材质、噪声、丢点、多径、玻璃、雨雾、动态模型和 mesh 碰撞细节。

更重要的是，雷达脚本订阅 `/odom/ground_truth` 来获得雷达的真实位姿。因此生成点云时使用的是 Gazebo 的精确位姿，而不是由机器人自身估计出来的位姿。

结论：可称为“基于解析射线求交的合成三维点云仿真”；不可称为“真实 MID360 高保真仿真”或“真实 MID360 数据”。

### P0-2：Nav2 的局部里程计直接来自 Gazebo 真值

相关文件：

```text
robots/configs/go2_config/launch/fast_lio_3d_localization.launch.py
robots/configs/go2_config/scripts/ground_truth_odom.py
robots/configs/go2_config/config/autonomy/fast_lio_nav2.yaml
```

`fast_lio_3d_localization.launch.py` 启动底层仿真时硬编码：

```python
"ground_truth_odom": "true"
```

`ground_truth_odom.py` 读取 `/odom/ground_truth`，再直接发布：

```text
/odom
odom → base_footprint
```

而 Nav2 参数明确使用 `/odom`。因此当前 Nav2 的短时运动估计和局部代价地图坐标来自完美的 Gazebo ground truth，并非仅由 FAST-LIO 或真实轮/腿部里程计获得。

ICP 仍会计算 `map → odom`，但“odom”这一支本身过于理想。该结果只能验证接口、TF 桥接和规划控制流程，不能证明真实里程计漂移下的导航鲁棒性。

### P0-3：ICP 不是全局重定位

相关文件：

```text
robots/configs/go2_config/scripts/pcd_icp_localizer.py
```

定位器必须先收到 RViz `/initialpose`，以此作为 ICP 初值。初值过远、方向相差过大、环境重复或点云不足时，ICP 可收敛到错误局部解或拒绝结果。

它没有全局候选搜索、回环、失效恢复、kidnapped-robot recovery，也没有经过系统性的定位精度评估。当前所谓“重定位”应表述为：**给定大致初始位姿后的局部 PCD-ICP 配准**。

### P0-4：三维仅用于点云/定位；Nav2 仍是二维

Nav2 使用 `mid360_3d_nav.yaml/.pgm` 和 `/scan`，只规划平面 `x、y、yaw`。`pcd_icp_localizer.py` 在构建 `map → odom` 时主动剥离 roll/pitch，以保持二维 costmap 水平。

因此不能表述为“三维路径规划”“利用三维高度避障”或“可导航楼梯/台阶/坡面”。项目是 **3D 建图与定位 + 2D Nav2 平面导航**。

## 4. 中优先级技术债与已知问题

### P1-1：已经拉取 `livox_laser_simulation_ros2`，但当前运行没有使用它

`robot_VLP.xacro` 引入了：

```text
$(find ros2_livox_simulation)/urdf/mid360.xacro
```

但该 `mid360.xacro` 实际只创建了 `mid360_link` 和固定关节，并在注释中说明扫描由 `go2_config/fast_mid360_scan.py` 生成；当前启动的实际节点则是 `fast_mid360_3d.py`。

所以仓库内存在 Livox Gazebo 仿真依赖、MID360 xacro、旧的 `fast_mid360_scan.py` 和当前 `fast_mid360_3d.py` 多套路径，容易让人误以为已经启用了 Livox Gazebo 插件。交接后应统一成一条明确的数据来源，删除或标记不用的路径。

### P1-2：地图产生过程和 2D 投影过程没有形成可复现的实验记录

虽然有 `pcd_to_nav2_map.py`，但当前 `mid360_3d.pcd`、`mid360_3d_nav.pgm` 和 `mid360_3d_nav.yaml` 没有附带“由哪次建图、什么参数、什么机器人路线、何时生成”的元数据或版本记录。

必须补充：原始 PCD、生成二维地图命令、参数、日期、world 版本和截图；否则更换地图或答辩追问时难以复现。

### P1-3：TF 架构是自定义桥接，未做系统性验证

定位器同时广播：

```text
map → camera_init
map → odom
```

FAST-LIO 使用 `camera_init → body`，CHAMP/仿真使用 `odom → base_footprint → base_link`。代码假设 FAST-LIO 的 `body` 原点与 `base_link` 一致，再用矩阵补偿高度和二维 odom。

这能让当前 Nav2 工作，但 `body` 与 `base_link` 并非一条由 URDF 明确发布的标准 TF 链。需要用 `view_frames`、`tf2_echo` 和实际移动实验验证不存在跳变、重复父节点、时间戳错配或地图/机器人可视化错位。

### P1-4：启动靠固定延时，不靠健康检查

`fast_lio_nav2_demo.launch.py` 用固定 `TimerAction`：约 16 秒启动 map server、18 秒启动 Nav2、22 秒启动 RViz。

机器性能较慢、Gazebo 载入较慢或 FAST-LIO 没有数据时，Nav2 仍可能提前启动，出现 `map` TF 不存在、队列堆满或无法规划。应改成根据控制器、点云、FAST-LIO odom 和 ICP bridge 就绪事件启动。

### P1-5：启动/图形稳定性依赖 WSLg 环境

本机曾发生 `gzclient` 和 RViz 进程启动但窗口不显示；重启 WSL 后恢复。常用环境变量：

```bash
export GAZEBO_MODEL_DATABASE_URI=""
export QT_QPA_PLATFORM=xcb
```

必要时可加 `LIBGL_ALWAYS_SOFTWARE=1`，但会降低性能。原生 Ubuntu 22.04 一般更稳定。

### P1-6：外部依赖与可移植性仍有风险

`livox_ros_driver2` 在构建时需要系统 `/usr/local/lib/liblivox_lidar_sdk_shared.so`。交付脚本会尝试编译安装 Livox-SDK2，但仍依赖 sudo、apt、ROS 2 Humble/Gazebo 11/系统库版本。

当前源码工作区是 dirty 状态，很多关键文件未提交到 Git。没有锁定依赖版本、commit hash、Docker 镜像或 CI。交接前应至少建立 Git 分支、提交全部可用文件、写 tag。

### P1-7：存在运行时警告，尚未根治

已观察到：

- `gazebo_ros2_control`：`Parameter 'hold_joints' has already been declared`；当前不阻塞启动，但原因未彻底定位；
- RViz/Nav2 在 ICP 初始位姿前会报 `map` frame 不存在、消息过滤队列满；初始位姿与 ICP 成功后才正常；
- 按 `Ctrl+C` 时，部分 Python 节点可能出现重复 `rclpy.shutdown()` 异常；属于退出清理问题；
- Nav2 行走可能出现“停一下再走”的表现，尚未做控制频率、costmap、速度平滑器和 CHAMP 步态参数的定量调优。

### P1-8：配置仍有绝对路径残留

`config/fast_lio_mid360_sim.yaml` 中仍有：

```text
map_file_path: /home/ice/go2_ws/maps/mid360_3d.pcd
```

当前 `fast_lio_3d_mapping.launch.py` 会用 launch 参数覆盖它，因此常规启动可用；但若直接运行 FAST-LIO 或换启动文件，仍可能写到错误路径。应删除此绝对路径，统一使用 launch 参数或包/用户目录展开。

## 5. 已做的修改中，哪些是正当修复，哪些仍需回归验证

以下改动用于让仿真能启动和控制器能加载：

- `champ_gazebo/launch/gazebo.launch.py`：把立即执行的 `ros2 control load_controller` 改为等待 `/controller_manager` 的官方 `controller_manager/spawner`；
- `robot_VLP.xacro`：将旧 VLP/Velodyne 配置改为 `mid360_link` 固定安装位姿；
- `quadruped_controller.*`：增加 `/cmd_vel/smooth` 超时清零，避免命令停止后机器人继续走；
- `pcd_icp_localizer.py`：自定义 3D ICP、`map → odom` 平面桥接；
- 多个 launch/config：加入 FAST-LIO、Nav2、地图和 RViz 启动链。

这些改动没有完整单元测试、回归测试或真机测试。特别是控制器、TF 和地图桥接不能视为已完成工程化验证。

## 6. 当前文件与入口索引

| 目的 | 关键文件 |
| --- | --- |
| 总入口 | `tools/run_fast_lio_nav2_demo.bash` |
| 三维建图入口 | `tools/run_fast_lio_3d_mapping.bash` |
| Gazebo + 自写雷达 | `robots/configs/go2_config/launch/gazebo_mid360.launch.py` |
| 自写三维射线点云 | `robots/configs/go2_config/scripts/fast_mid360_3d.py` |
| Ground-truth odom | `robots/configs/go2_config/scripts/ground_truth_odom.py` |
| FAST-LIO 参数 | `robots/configs/go2_config/config/fast_lio_mid360_sim.yaml` |
| ICP 定位与 TF bridge | `robots/configs/go2_config/scripts/pcd_icp_localizer.py` |
| 3D 重定位 launch | `robots/configs/go2_config/launch/fast_lio_3d_localization.launch.py` |
| Nav2 总 launch | `robots/configs/go2_config/launch/fast_lio_nav2_demo.launch.py` |
| Nav2 参数 | `robots/configs/go2_config/config/autonomy/fast_lio_nav2.yaml` |
| 3D 世界 | `robots/configs/go2_config/worlds/mid360_mapping.world` |
| PCD 转 2D 图工具 | `robots/configs/go2_config/scripts/pcd_to_nav2_map.py` |

## 7. 对下一位开发者的建议顺序

1. **先提交并冻结当前可运行版本**：清理 `__pycache__`，提交所有 launch、脚本、地图、配置和 README，记录依赖版本。
2. **移除 ground truth 泄漏**：让 Nav2 `/odom` 来自可控的真实估计链，至少做“带噪声/漂移”的模拟；不要把 `/odom/ground_truth` 直接改名为 `/odom`。
3. **替换或升级雷达模型**：明确使用 `livox_laser_simulation_ros2` 的可工作的传感器插件，或者继续维护解析射线模型但加入真实扫描模式、噪声、材质、动态障碍和可配置参数。
4. **重新调 FAST-LIO**：匹配新的雷达速率、线束、时间戳、IMU、外参和噪声模型。
5. **整理 TF**：明确唯一的机器人基座链和 map/odom 发布者，用 `tf2_tools view_frames` 回归测试。
6. **让 Nav2 等待就绪**：以事件/服务/话题健康状态替代固定 16/18/22 秒延时。
7. **做可量化实验**：记录定位误差、ICP 成功率、重定位初值范围、规划成功率、到达误差、速度和 CPU 占用。
8. 最后再考虑真机接入：真实 MID360 驱动、时间同步、IMU、外参标定、安全限速和急停。

## 8. 演示与报告可说/不可说

可以说：

- 完成了 Gazebo 中 Go2 的三维合成点云、FAST-LIO、PCD ICP 和 Nav2 接口链路验证；
- 使用解析射线求交生成兼容 Livox 消息格式的合成点云；
- 完成 3D 地图上的局部 ICP 配准，以及到二维 Nav2 的 TF 桥接。

不应说：

- “已真实仿真 MID360”或“使用真实 MID360 点云”；
- “完全依靠激光实现自主导航”；
- “实现了三维路径规划”；
- “实现了无需初始位姿的全局重定位”；
- “已在真机验证”。

## 9. 交接时应附带的材料

- 当前源码工作区或交付 ZIP；
- 本文件；
- `README_MID360.md`、`README_CURRENT_PROGRESS.md`；
- 可运行的地图文件；
- Gazebo、RViz、ICP accepted、Nav2 Goal 成功的录屏；
- `git status --short` 和所有依赖仓库的 commit hash；
- 如要继续维护，必须给出下一步选择：高保真 Gazebo 插件还是先接真实 MID360。

