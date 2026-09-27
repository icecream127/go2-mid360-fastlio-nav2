# Unitree Go2 + Livox MID-360 + FAST-LIO + Nav2

> **真实 MID-360 当前使用 Xju 新版 FAST-LIO：请按 [真实雷达使用与参数入口](README_REAL_MID360.md) 操作。**
> 旧真实雷达操作记录已归档；下面的仿真导航说明为原有流程，新算法坐标系改变后尚未回归验收。
> 本轮代码、WSL 时钟修改和未完成的验收见 [2026-09-27 变更记录](docs/CHANGELOG_2026-09-27.md)。

本项目在 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Classic 11 中仿真 Unitree Go2 与 Livox MID-360，并提供一套已经建好的地图。原流程可启动 Gazebo 和 RViz，进行三维点云定位与 Nav2 目标点导航，不需要先自行建图；本轮更换 FAST-LIO 依赖后，仿真端到端运行尚待重新验证。

## 演示视频

![Go2 + MID-360 + FAST-LIO + Nav2 仿真演示](docs/demo/go2_mid360_fastlio_nav2_demo.gif)

上图展示了项目在 Gazebo 和 RViz 中的实际运行效果，包括已有地图定位和 Nav2 目标点导航。

## 一、普通部署与使用

### 1. 环境要求

- Ubuntu 22.04（原生 Ubuntu 或 WSL2 均可）
- ROS 2 Humble Desktop
- 能正常显示 Gazebo 和 RViz 图形窗口
- 安装过程需要网络和 `sudo` 权限

### 2. 下载并自动安装

打开 Ubuntu 终端，依次执行：

```bash
sudo apt update
sudo apt install -y git

mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/icecream127/go2-mid360-fastlio-nav2.git unitree-go2-ros2

cd unitree-go2-ros2
./tools/setup_mid360_dependencies.bash
```

安装脚本会自动完成以下工作：

- 安装项目需要的 ROS 2 和系统依赖；
- 下载固定版本的 Livox SDK2、Livox ROS 2 驱动、Livox Gazebo 插件和 FAST-LIO；
- 下载固定版本的 CPU `fast_gicp`，供独立的真实雷达 GICP 定位包构建；
- 应用 ROS 2 Humble 兼容补丁；
- 编译整个 `~/go2_ws` 工作空间；
- 把仓库附带的地图复制到 `~/go2_ws/maps`。

### 3. 一条命令启动

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_nav_bringup navigation.launch.py gui:=true show_rviz:=true
```

等待 Gazebo 和 RViz 窗口出现；系统会先让机器狗站稳，再初始化 FAST-LIO。也可以继续使用 `./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash`，它会自动加载上述环境并调用同一个启动文件。

如果已有其他 Gazebo 或同类 ROS 2 仿真正在运行，请先将其关闭，避免端口、节点名和话题冲突。

导航不会调用建图启动文件：它启动仿真、FAST-LIO 里程计、ICP 定位和 Nav2。
FAST-LIO 的程序名仍为 `fastlio_mapping`，导航中的节点名为 `fastlio_odometry`；
算法内部仍维护里程计匹配所需的局部地图，但不发布 FAST-LIO 地图、不保存 PCD，
也不会覆盖已有参考地图。需要主动建图时才使用 `go2_nav_bringup mapping.launch.py`。

#### 调参入口与地图选择

- 默认读取当前工作空间的 `maps/mid360_3d.pcd` 和 `maps/mid360_3d_nav.yaml`，
  启动日志会打印实际路径。包中的地图是安装时的分发副本，修改它不会自动替换工作空间地图。
  换地图时显式传入 `pcd_map:=/绝对路径/地图.pcd nav_map:=/绝对路径/地图.yaml`，
  两者必须来自同一坐标系。
- `initial_x / initial_y / initial_yaw` 是地图坐标系下的 ICP 初猜；
  `world_init_x / world_init_y / world_init_z / world_init_heading` 是 Gazebo 出生位姿。
  两组参数不能默认照抄，朝向单位都是弧度。
- 点云转扫描配置为 `go2_mid360_sim/config/mid360_to_scan.yaml`，
  节点名为 `mid360_cloud_to_scan`。启动参数 `use_sim_time` 会覆盖该 YAML 中的时间设置。
  当前按雷达坐标系上方 0.02～1.50 m 取截面，仅针对墙柱平地演示，
  不能保证低矮障碍物检出，也不是通用地面分割。
  本轮已发现空扫描帧及定位精度待验证项，详见 [验收记录](docs/NAVIGATION_PARAMETER_AUDIT.md)。
- 当前使用 ICP，不启动 AMCL；导航参数文件中已移除未使用的 AMCL 段。
- 路径控制器、恢复旋转、速度平滑器和 CHAMP 的角速度上限统一为 0.5 rad/s。
  前三项位于 `go2_nav_bringup/config/autonomy/fast_lio_nav2.yaml`，
  CHAMP 位于 `robots/configs/go2_config/config/gait/gait.yaml`。
- 仿真关节 PID 的有效入口为
  `robots/descriptions/go2_description/config/ros_control/ros_control.yaml`，
  由模型中的 Gazebo 插件加载；不是 `go2_config` 下的同名文件。
  修改参数后请重新编译对应包并重启相关节点。

启动脚本默认使用 Fast DDS 共享内存传输，以免高流量点云影响 Nav2 生命周期服务。如果所在环境不支持共享内存，可在启动命令前设置 `GO2_DDS_TRANSPORT=udp` 切换为 UDP。

### 4. 在 RViz 中导航

1. 确认 Gazebo 中已经出现机器狗；
2. 等待终端连续出现 `ICP accepted`，再等待 Nav2 完成启动（整个过程通常约半分钟）；
3. 选择 **Nav2 Goal**，在地图可通行区域拖出目标位置和朝向；
4. 机器狗开始行走，RViz 左下角出现 `Feedback: reached` 表示到达目标。

默认场景会自动使用 `x=0、y=0、yaw=0` 作为 ICP 初始估计，不需要点击 **2D Pose Estimate**。当前使用的是 FAST-LIO + ICP 重定位，因此 RViz 面板显示 `Localization: inactive` 不代表定位失败，应以 `ICP accepted`、点云与地图对齐情况及导航结果为准。

在启动仿真的终端按 `Ctrl+C` 即可停止。

## ROS 2 包结构与开发

| 功能包 | 职责 |
| --- | --- |
| `go2_mid360_sim` | MID-360 Gazebo 启动、传感器参数和场景 |
| `go2_fastlio_localization` | FAST-LIO 建图启动、ICP 节点、里程计桥接和地图转换 |
| `go2_real_localization` | 真实 MID-360 的已有 PCD 地图 GICP 重定位；[单独使用说明](go2_real_localization/README.md) |
| `go2_nav_bringup` | Nav2 总启动、导航参数、示例地图和 RViz |
| `go2_config` | 原有 Go2 步态/关节配置，以及旧命令兼容入口 |
| `go2_description`、`champ_*` | 原有机器人描述和第三方运动控制 |

Python 节点通过 `ament_python` 的入口安装；仿真、导航启动包通过 `ament_cmake` 安装资源。修改包后，在工作空间重新构建并加载环境：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select go2_config go2_mid360_sim go2_fastlio_localization go2_nav_bringup
source install/setup.bash
ros2 launch go2_nav_bringup navigation.launch.py gui:=true show_rviz:=true
```

建图入口为 `ros2 launch go2_nav_bringup mapping.launch.py`，定位入口为 `ros2 launch go2_nav_bringup localization.launch.py`。上述入口默认使用同一套仿真、定位参数；`tools/run_*.bash` 保留为快捷入口。旧 `go2_config` 主启动文件会转发到新包。

旧解析式点云和真值辅助脚本仍保留在 `go2_config/scripts` 中，仅用于历史实验兼容；默认导航入口不启用它们。自定义包从原工程迁移的文件继续保留原有版权与许可，第三方代码及许可证仍位于原目录。

## 二、项目功能

- 在 Gazebo Classic 11 中仿真 Unitree Go2 四足机器人；
- 使用 Gazebo 射线传感器和 Livox 插件模拟 MID-360 风格的非重复扫描点云；
- 发布 `/livox/lidar`（`PointCloud2`）和 `/livox/lidar_custom`（Livox `CustomMsg`）；
- 使用 FAST-LIO 输出三维激光惯性里程计和三维 PCD 地图；
- 使用已有 PCD 地图和实时三维点云进行 ICP 重定位；
- 将三维点云转换为 `/scan`，供 Nav2 二维代价地图避障；
- 使用 Nav2 完成路径规划、目标点导航和避障；
- 提供默认 Gazebo 场景对应的三维地图与二维导航地图。

导航使用的主要 TF 坐标链为：

```text
map -> odom -> base_footprint -> base_link -> mid360_link
```

`/odom` 由 FAST-LIO 的 `/Odometry` 转换得到。默认场景使用已知出生点作为 ICP 初始估计，主定位和导航流程不依赖 Gazebo 的 `/odom/ground_truth` 真值里程计。

## 三、使用已有地图或自行建图

### 直接使用仓库地图

仓库已经提供以下文件：

```text
mid360_3d.pcd
mid360_3d_nav.pgm
mid360_3d_nav.yaml
```

默认启动命令会自动使用这些地图。导航模式不会覆盖已有地图。

这些地图只对应项目自带的 `mid360_mapping.world` 场景。更换 Gazebo 世界后，需要为新场景重新生成 PCD 地图和二维导航地图。

### 自行进行三维建图

下面的流程不会自动覆盖仓库附带的 `mid360_3d.pcd`。当前 Xju FAST-LIO
检出的关机保存缓冲没有填充，**按 Ctrl+C 不会可靠地保存 PCD**；
必须在建图节点仍运行时另行保存，并确认工具打印 `SAVED`。

启动三维建图：

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash gui:=true show_rviz:=true
```

另开一个终端，用键盘控制机器狗：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

完成环境扫描后，保持建图终端运行，在另一个已 `source` 工作空间的终端保存：

```bash
ros2 run go2_fastlio_localization save_live_map
```

工具默认写入 `~/go2_maps/real_room_时间戳.pcd`，并打印准确的 `SAVED` 路径。
确认文件存在且点数合理后，再在建图终端按 `Ctrl+C`。文件名中的
`real_room` 是保存工具的通用命名，在这里不代表该数据来自真实雷达。

把刚打印的 PCD 路径代入下列命令，转换为 Nav2 二维地图：

```bash
ros2 run go2_fastlio_localization pcd_to_nav2_map \
  /home/ice/go2_maps/real_room_实际时间戳.pcd /home/ice/go2_maps/my_room_nav \
  --resolution 0.05 --z-min 0.35 --z-max 1.50 --padding 0.30 \
  --min-points 2 --dilation-cells 1
```

转换完成后，启动导航时显式传入新 PCD 的 `pcd_map` 和新 YAML 的
`nav_map`，不要误用工作空间默认的旧地图。二维投影需要人工检查障碍物和
未观测区域；未验证前不能将它当成实机安全导航地图。

也可以在启动时指定其他地图：

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash \
  gui:=true show_rviz:=true \
  pcd_map:=$HOME/go2_ws/maps/example.pcd \
  nav_map:=$HOME/go2_ws/maps/example_nav.yaml \
  initial_x:=1.0 initial_y:=2.0 initial_yaw:=0.0
```

把示例中的 `initial_x`、`initial_y` 和 `initial_yaw` 改成新地图中机器狗的固定出生位姿。若出生位置不固定，可增加 `auto_initial_pose:=false`，然后在 RViz 中使用 **2D Pose Estimate**。PCD、PGM 和 YAML 必须来自同一个场景，并且坐标原点需要保持一致。

## 四、已知限制

- 当前是 Gazebo 中的 MID-360 传感器仿真，不是真实 MID-360 硬件，也不是硬件级精度的数字孪生；
- ICP 仍需要相对合理的初始估计；在出生位姿改变、场景高度对称、特征太少或地图与环境差异很大时，可能无法正确收敛；
- FAST-LIO 和 ICP 使用三维点云定位，但 Nav2 的路径规划与局部避障仍基于二维地图和二维 `/scan`；
- WSL2 的 Gazebo/RViz 显示依赖 WSLg 和显卡驱动，图形性能通常低于原生 Ubuntu。

## 五、开源项目说明

本项目基于 Unitree Go2 description、CHAMP、Livox SDK2、`livox_ros_driver2`、LCAS `livox_laser_simulation_ros2`、XjuHurricaneQuadVision `FAST_LIO` 和 `fast_gicp` 等开源项目开发，各部分遵循其原始许可证。

`patches/` 保存针对固定依赖版本的补丁；当前安装脚本应用 Gazebo Livox、
真实 Livox 驱动时间戳、FAST-LIO 手持建图与时间戳保护补丁。
`fast_lio_local_map.patch` 是历史局部地图实验，不在当前累计建图入口应用。
当前保存方式见上文，不承诺退出 FAST-LIO 时自动生成完整 PCD。

## 六、Docker 部署

Docker 方式适合希望隔离 ROS 依赖，或者需要把相同运行环境交给其他人的情况。Docker 镜像内部已经包含 Ubuntu 22.04、ROS 2 Humble、Livox 依赖、FAST-LIO 和本项目工作空间。

### Windows 10/11 + WSL2 准备

1. 安装并启动 Docker Desktop；
2. 启用 **Use the WSL 2 based engine**；
3. 在 **Settings → Resources → WSL Integration** 中启用要使用的 Ubuntu 22.04；
4. 在 Ubuntu 终端确认 Docker 可用：

```bash
docker version
docker compose version
```

原生 Ubuntu 22.04 可以直接安装 Docker Engine 和 Docker Compose，不需要 WSL Integration。

### 下载项目

如果还没有克隆项目：

```bash
mkdir -p ~/go2_docker
cd ~/go2_docker
git clone https://github.com/icecream127/go2-mid360-fastlio-nav2.git
cd go2-mid360-fastlio-nav2
```

如果已经按照普通部署克隆过项目，直接进入原仓库：

```bash
cd ~/go2_ws/src/unitree-go2-ros2
```

### 构建镜像

```bash
docker compose build
```

首次构建会下载 ROS 镜像、依赖源码并完成编译，耗时较长。最终镜像约为 2 GB。

镜像会构建完整工作空间，并检查 `go2_mid360_sim`、
`go2_fastlio_localization`、`go2_real_localization` 和 `go2_nav_bringup`
是否可被 ROS 2 找到；这不是实机 GICP 或 Gazebo 图形运行验收。
如果使用的是拆包前构建的旧镜像，更新仓库后执行：

```bash
git pull --ff-only
docker compose up --build
```

该命令会重建并启动更新后的容器，保留地图数据卷。容器直接运行
`ros2 launch go2_nav_bringup navigation.launch.py`，由入口脚本加载 ROS 和 Livox 库环境。

### 启动导航仿真

```bash
docker compose up
```

仅运行后台仿真、不打开窗口时：

```bash
GO2_GUI=false GO2_RVIZ=false docker compose up
```

也可以用同一镜像启动建图：

```bash
docker compose down
docker compose run --rm go2-nav ros2 launch go2_nav_bringup mapping.launch.py gui:=true show_rviz:=true
```

建图会使用相同的地图卷，操作前请备份已有地图。参考地图位于镜像中
`go2_nav_bringup` 的安装资源目录；空地图卷会自动初始化，已有地图不会因为
重建镜像而被覆盖。要使用自己后续建的新地图，需要同步成套的 PCD、PGM 和 YAML。

Gazebo 和 RViz 出现后，按照以下顺序操作：

```text
等待 ICP accepted -> Nav2 Goal
```

停止时在当前终端按 `Ctrl+C`，然后执行：

```bash
docker compose down
```

不要随意执行 `docker compose down -v`，因为 `-v` 会同时删除保存地图的 `go2-maps` 数据卷。

当前项目运行不需要 CUDA。默认使用软件 OpenGL，以提高 WSLg 下 Gazebo 和 RViz 的兼容性。如果以后加入了需要 CUDA 的算法，并且 Docker Desktop 已启用 NVIDIA GPU，可使用：

```bash
docker compose -f compose.yaml -f compose.cuda.yaml up
```
