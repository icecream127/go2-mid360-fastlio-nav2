# 项目目录与真实 MID-360 启动入口

2026-09-22 整理。日常真实雷达操作以本文为准，早期排查记录用于追溯。

## 1. 每个终端先设置环境

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
```

WSL 当前有 Wi-Fi、有线口同属 192.168.1.0/24 的路由冲突。
先 `ip route get 192.168.1.111`；应走雷达有线口，当前是 eth3、源地址 192.168.1.50。
必要时 `sudo ip route replace 192.168.1.111/32 dev eth3 src 192.168.1.50`。
接口名需按实际机器核实；该路由重启后可能丢失。此问题与 launch 命名无关。

## 2. 推荐分开启动，便于学习与排错

终端一（只启动真实驱动）：

```bash
ros2 launch go2_fastlio_localization real_lidar.launch.py
```

终端二（只启动 FAST-LIO 和 RViz）：

```bash
ros2 launch go2_fastlio_localization real_fastlio.launch.py
```

无图形界面时第二条加 `show_rviz:=false`。雷达初始化期间保持静止。
另一种等价的单命令入口是：

```bash
ros2 launch go2_fastlio_localization real_mapping.launch.py
```

二选一，不能把两种方式叠加运行，否则会启动重复驱动/算法。
一键入口复用 real_lidar.launch.py；real_fastlio 是同一建图入口关闭驱动后的薄封装，
没有复制另一份算法参数。直接 launch 不带旧 shell 脚本的进程防重检查。

## 3. 到底改哪个文件

以下路径以 src/unitree-go2-ros2 为根：

| 要做的事 | 唯一推荐编辑位置 |
| --- | --- |
| 真实雷达/主机 IP、接收端口 | go2_fastlio_localization/config/mid360_real.json |
| 真实 FAST-LIO 体素、外参、输入话题 | go2_fastlio_localization/config/fast_lio_mid360_real.yaml |
| 真实建图显示 | go2_fastlio_localization/config/real_mapping.rviz |
| 驱动参数：CustomMsg、10 Hz、frame_id | go2_fastlio_localization/launch/real_lidar.launch.py |
| FAST-LIO 与 RViz 参数、保存路径 | go2_fastlio_localization/launch/real_mapping.launch.py |
| 保存点云代码 | go2_fastlio_localization/go2_fastlio_localization/save_live_map.py |
| 静止检查代码 | go2_fastlio_localization/go2_fastlio_localization/check_stationary.py |

ROS 实际加载 install/share 中的配置，采用 symlink-install 时通常指向源码。
不要分别修改 src 和 install 维护两套配置；重编译并 source 后重启相关节点。

## 4. 学长给的两个命令如何对应

### Livox 驱动

`ros2 launch livox_ros_driver2 msg_MID360_launch.py` 在本机存在，是可用的上游启动方式。
它读取依赖包的 `config/MID360_config.json`，不是本项目的 `mid360_real.json`。
本机这两份 JSON 当前 IP 均为主机 .50、雷达 .111，但以后只改其中一个就会分叉。
上游该 launch 把 JSON 路径写在 Python 文件中，没有声明 user_config_path launch 参数，
因此不能假定命令后追加 user_config_path 就可以覆盖。

本项目 real_lidar 启动同一个 livox_ros_driver2_node，使用同类关键参数：
xfer_format=1、publish_freq=10、frame_id=livox_frame。它额外暴露 user_config_path 参数，
让配置在项目仓库内管理。一键组合 launch 也是正常 ROS 2 用法。

### FAST-LIO

本机 Ericsii/FAST_LIO_ROS2 版本没有 mapping_mid360.launch.py；实际是：

```bash
ros2 launch fast_lio mapping.launch.py
```

默认会读取依赖包 config/mid360.yaml，因此直接运行不会使用本次真实建图调参。
若已有一个驱动运行，希望直接使用上游算法入口且加载本项目参数，可执行：

```bash
ros2 launch fast_lio mapping.launch.py \
  config_path:="$(ros2 pkg prefix --share go2_fastlio_localization)/config" \
  config_file:=fast_lio_mid360_real.yaml \
  rviz_cfg:="$(ros2 pkg prefix --share go2_fastlio_localization)/config/real_mapping.rviz" \
  use_sim_time:=false
```

这个上游入口没有本项目的 save_pcd/map_output 覆盖逻辑：YAML 默认 pcd_save_en=true，
map_file_path 是相对路径 real_mid360_room.pcd，退出可能写入当前目录并覆盖同名文件。
日常推荐第 2 节入口，默认关闭自动保存，使用下一节的时间戳另存。
不能同时运行上游算法入口与 real_fastlio/real_mapping。

## 5. 保存和退出

保持建图运行，在设置了相同 ROS 环境的另一终端执行：

```bash
ros2 run go2_fastlio_localization save_live_map
```

默认输出 ~/go2_maps/real_room_时间戳.pcd。等待收到累计 /Laser_map，最长 45 秒；
当前工具等待期间没有进度提示。环境不同会超时，不应仅凭沉默判断卡死。
保存成功后，分别在算法、驱动终端 Ctrl+C；一键启动则只需关闭其启动终端。
若原始驱动使用 xfer_format=0，FAST-LIO 的 CustomMsg 输入链无法使用，需改回 1。

## 6. 整个项目各部分

| 目录 | 责任 | 本次真实房间建图是否需要 |
| --- | --- | --- |
| go2_fastlio_localization | 建图入口、里程计桥接、ICP、地图转换、保存 | 使用其中真实建图部分 |
| go2_mid360_sim | Gazebo 雷达/场景仿真 | 否 |
| go2_nav_bringup | Nav2 导航、导航配置和示例地图 | 否 |
| champ、champ_teleop、robots 等 | 四足模型、控制及原有机器人配置 | 否 |
| tools | 安装、兼容 shell 入口和辅助检查 | 安装时使用，日常优先 ros2 命令 |
| patches | 本项目对 FAST-LIO 等依赖的补丁 | 安装依赖时应用，不要重复手工应用 |
| docs | 操作说明、历史排查记录 | 当前入口看本文，旧文用于追溯 |
| ../FAST_LIO_ROS2 | 外部算法源码 | 是 |
| ../livox_ros_driver2、../Livox-SDK2 | 外部真实驱动和 SDK | 是 |

## 7. 看起来重复的文件如何处理

- config/fast_lio_mid360_sim.yaml 是仿真专用，保留；不要拿它覆盖真实 YAML。
- 依赖包 FAST_LIO_ROS2/config/mid360.yaml 与 livox_ros_driver2/config/MID360_config.json
  是上游入口默认配置，保留，但推荐操作不编辑它们。
- ~/go2_ws/real_mid360 下 raw.launch.py、raw.rviz、run_raw.bash、MID360_config.json 和
  save_live_map.py 是前序本机脚本。保留兼容/排查，不是部署必需项；不要混用启动。
- tools/run_real_mid360_mapping.bash 是兼容入口，最终调用同一个 ROS launch。
- tools/check_fastlio_stationary.py 是旧检查副本，日常使用
  `ros2 run go2_fastlio_localization check_stationary`。
- bags、maps、~/go2_maps 是数据；build/install/log 是生成物，都不是需要塞进源码包的功能代码。

本轮不删除旧文件与地图，先将入口和配置归属统一。历史记录不当作当前操作指南。

## 8. 编译与迁移

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_fastlio_localization --symlink-install
source install/setup.bash
```

Ubuntu 22.04 小电脑安装 ROS 2 Humble，按主 README 安装依赖，在目标机器重新编译。
修改本项目 mid360_real.json 为目标网卡和雷达地址，核对路由，再用第 2 节启动。
完整依赖安装脚本可能同时安装仿真依赖，当前未拆成最小硬件部署依赖集。
不要复制 WSL build/install 到新机器。

这次整理只复用启动逻辑并补充入口，没有改变算法精度参数。
短时离线漂移改善不等于房间绝对精度达到 1 cm，详见历史排查 README。
