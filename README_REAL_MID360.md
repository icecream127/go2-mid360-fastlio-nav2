# 真实 MID-360 建图：当前唯一操作入口

2026-09-22：已切换到 XjuHurricaneQuadVision/FAST_LIO，提交
`440a8e3e909023b7cb084e99b0c5070e3baebe86`。当前算法源码不套用旧版补丁。

本次恢复累计地图输出：FAST-LIO 每秒将当前配准扫描变换到 odom 后追加，/Laser_map 和保存工具得到本次启动以来的累计点云，适合覆盖整间房。累计只改变地图输出，不修正位姿；地图随时间增长，发布、RViz 和保存负载会增加。

## 源码在哪里

```text
go2_ws/src/
├── FAST_LIO/                         # 新版 FAST-LIO 原始仓库
├── livox_ros_driver2/                # 真实雷达驱动
├── Livox-SDK2/                      # 驱动依赖
└── unitree-go2-ros2/
    ├── README_REAL_MID360.md         # 本文：真实雷达看这里
    ├── go2_fastlio_localization/
    │   ├── launch/real_lidar.launch.py    # 只启动驱动
    │   ├── launch/real_fastlio.launch.py  # 调用新版上游 mapping.launch.py
    │   ├── launch/real_mapping.launch.py  # 可选：组合前两个入口
    │   ├── config/mid360_real.json       # 唯一真实网络配置
    │   ├── config/fast_lio_mid360_real.yaml # 唯一真实算法配置
    │   ├── config/real_mapping.rviz      # 唯一真实建图显示配置
    │   └── go2_fastlio_localization/
    │       ├── save_live_map.py
    │       └── check_stationary.py
    ├── go2_mid360_sim/               # 仿真，非真实建图入口
    ├── go2_real_localization/         # 已有 PCD 上的实机 GICP 定位，非建图入口
    └── go2_nav_bringup/              # 导航，非真实建图入口
```

只在上述三个 config 文件中调真实雷达，不同时修改依赖包的示例 YAML/JSON。
FAST_LIO/config/mid360.yaml、livox_ros_driver2/config/MID360_config.json 是上游示例，
我们的入口明确传入本项目配置。依赖源码保留示例文件是正常的，不代表同时生效。

## 启动

每个终端先执行：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
```

终端一：

```bash
ros2 launch go2_fastlio_localization real_lidar.launch.py
```

终端二：

```bash
ros2 launch go2_fastlio_localization real_fastlio.launch.py
```

无桌面加 `show_rviz:=false`。雷达启动时放稳，等初始化完成再移动。
可选一键入口是 `ros2 launch go2_fastlio_localization real_mapping.launch.py`，
与分开启动二选一，不要重复启动。

新版同样没有 `mapping_mid360.launch.py`，实际文件是 `mapping.launch.py`。
real_fastlio 只是将正确配置传给 `ros2 launch fast_lio mapping.launch.py` 的薄封装。
real_lidar 启动原版 `livox_ros_driver2_node` 并传入上述网络 JSON。
上游 `ros2 launch livox_ros_driver2 msg_MID360_launch.py` 也能启动驱动，
但读取的是驱动包示例 JSON，日常不要交替使用两套配置。

## 参数接口

| 目标 | 文件/参数 | 当前设置 |
| --- | --- | --- |
| 主机接收地址 | mid360_real.json / host_net_info 各 IP | 192.168.1.50 |
| 雷达地址 | mid360_real.json / lidar_configs.ip | 192.168.1.111 |
| 点云格式 | real_lidar.launch.py / xfer_format | 1，CustomMsg |
| 发布频率 | real_lidar.launch.py / publish_freq | 10 Hz |
| 近距离盲区 | fast_lio_mid360_real.yaml / preprocess.blind | 0.8 m，过滤雷达坐标系内的近点 |
| 体素 | fast_lio_mid360_real.yaml / filter_size_surf、filter_size_map | 0.1 m（用户当前设置） |
| 点抽样 / 最大迭代 | point_filter_num / max_iteration | 2 / 3 |
| 局部窗口边长 | cube_side_length | 200 m；需大于 3 × det_range，不是雷达量程 |
| 探测距离参数 | mapping.det_range | 40 m，参与局部地图管理 |
| 扫描频率提示 | preprocess.scan_rate | 10 Hz，不是扫描距离 |
| 外参在线估计 | mapping.extrinsic_est_en | false；使用本配置给定的 MID-360 外参，避免在线估计漂动 |
| 外参 | mapping.extrinsic_T/R | 新仓库 MID360 默认值 |
| 自动保存 | pcd_save.pcd_save_en | false，手动另存 |

当前真实 YAML 的盲区为 0.8 m、体素为 0.1 m、点抽样为 2；近距离点过滤由手持补丁修正。
本项目的手持补丁将新算法的 IMU 初始化样本数从 10 调至 1000。静置等待终端出现 IMU Initial Done，再拿起雷达。
旧版的时间初始化补丁未套到新算法中；当前参数不代表已达到 1 cm 建图精度。

源码中窗口移动阈值为 1.5 * det_range，窗口边长应大于 3 * det_range。
所以 cube_side_length=40 配 det_range=40 会不合理。
det_range 本身并不是硬距离过滤，当前没有实现“严格删除所有超过 40 m 的点”；
要硬裁剪需另做保留逐点时间字段的过滤功能，不能靠改 scan_rate 实现。

## 话题与显示

| 名称 | 类型/含义 |
| --- | --- |
| /livox/lidar | livox_ros_driver2/msg/CustomMsg，真实输入 |
| /livox/imu | sensor_msgs/msg/Imu，真实输入 |
| /Odometry | nav_msgs/msg/Odometry，odom → livox_frame |
| /cloud_registered | PointCloud2，当前扫描在 odom 中的坐标 |
| /cloud_registered_body | PointCloud2，新源码标注 livox_frame |
| /Laser_map | PointCloud2，每秒发布从本次启动至今累计的已配准扫描，odom 坐标系 |
| /path | nav_msgs/msg/Path，运动轨迹 |
| /tf | odom → livox_frame，新版的坐标系命名 |

RViz 固定坐标系 odom。默认显示彩色累计地图和当前里程计位置，帧率 15。
白色当前扫描（Registered Cloud）、TF 和轨迹默认关闭，可按需单独勾选；
这是显示减负措施，不改变发布和保存内容。
连线是否出现不是建图精度判据，墙面在移动中是否对齐才更关键。
/Laser_map 每秒把当前已配准扫描追加到 pcl_wait_pub 并发布累计结果，保留机器人本次运行经过的区域。位姿漂移会累积为墙面重影；当前发布周期为 1 Hz，建议缓慢移动并在停止前保存。RViz 显示项名为 Accumulated Room Map；Decay Time 设为 0，因为消息本身已累计。
显示样式从 3 cm 的 Flat Squares 改为 2 像素 Points，保留原始点坐标。
地图受内部体素策略和局部窗口管理，不是固定点数上限，也不是每体素严格一个点。
当前窗口为 200 m，体素参数为 0.1 m；大范围走动仍会增长。更密的点云不能修正位姿漂移。
此次修复输入近点过滤与初始化，并将累计地图发布队列深度改为 1；没有增加动态物体剔除或回环。

## 手持建图

patches/fast_lio_handheld.patch 修复 FAST-LIO 的 CustomMsg 近点过滤条件：原代码中逻辑与的优先级导致部分盲区内的点仍进入配准。补丁还把 IMU 静置初始化阈值从 10 条样本改为 1000 条样本，并将 /Laser_map 发布队列从 20 改为 1，避免积压多帧越来越大的累计地图消息。

将雷达先放稳，等 IMU Initial Done 后平稳拿起；移动时避免快速旋转，尽量让墙面等固定结构持续出现在视野里。0.8 m 盲区只能过滤距离雷达小于 0.8 m 的点，人体更远的部分依然可能进入地图。

如果拿起后仍出现位置跳变，先核查 LiDAR 和 IMU 时间戳是否同步、点云逐点 offset_time 是否正常，再核查外参；不要靠继续增大盲区或调大体素掩盖问题。当前没有真实手持轨迹的精度验证。

## 保存

另一个已设置环境的终端：

```bash
ros2 run go2_fastlio_localization save_live_map
```

保存到 ~/go2_maps/real_room_时间戳.pcd。整间房走完后启动保存工具，等 SAVED 再 Ctrl+C 停止建图。工具需要运行中的 /Laser_map，默认最多等 45 秒；它保存下一条累计地图消息，ROS 环境不一致会超时。累计图不含回环优化；轨迹跳变会造成墙面重影。历史约 1 cm 记录是静止数据的离线短时相对漂移，不是整房绝对精度。
上游 /map_save 服务也写出累计点云；仍受 pcd_save_en 开关控制。
默认继续使用 save_live_map，不建议打开 pcd_save_en：原始扫描落盘缓冲是另一条增长路径。
旧 map_output、save_pcd 启动参数不再作为配置接口；保存通过此命令统一进行。

## 网络与部署

WSL 当前 Wi-Fi .226 与有线 .50 同网段，曾导致路由走错：

```bash
ip route get 192.168.1.111
# 先确认实际雷达接口，当前本机是 eth3，再按需执行：
sudo ip route replace 192.168.1.111/32 dev eth3 src 192.168.1.50
```

小电脑使用 Ubuntu 22.04 + ROS 2 Humble。重装依赖时 setup_mid360_dependencies.bash
已锁定新仓库和提交，自动应用手持建图及时间戳补丁。在目标机器重新编译，不能复制 WSL 的 install。
网络 JSON 改成该机实际 IP。代码全在 src；maps/bags 是数据，备份在 src 外，防止重复包。

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-select livox_ros_driver2 fast_lio go2_fastlio_localization --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

旧版和被撤下的重复脚本在 dependency_backups/xju_switch_20260922，可恢复。
该目录有 COLCON_IGNORE，不参与构建。不要把旧 FAST_LIO_ROS2 放回 src 与新版共存。
两个仓库都是 fast_lio 包名，colcon 不允许重复。

## 当前边界

本轮重点是替换真实雷达建图链。旧仿真/ICP/Nav2 源码保留，但此前依赖 camera_init/body；
新算法变为 odom/livox_frame，旧导航链需要单独适配和回归，不能宣称仍然兼容。
不要把旧 camera_init/body 的调参结论直接用于新版。没有证明新版本运动建图或绝对误差 ≤1 cm。
历史排查文档可能描述旧参数，当前操作仅以本文为准。

## 2026-09-25：时间戳倒退排查与修复

静止录包 `~/go2_ws/bags/mid360_stationary_test` 中，约 32 秒有一次 IMU
消息时间戳倒退 0.482 秒，录包接收时间也同时倒退约 0.482 秒；
相邻雷达帧之间积压了约 124 条 IMU（正常约 20 条）。这证明该次录包有
主机时间回拨/出帧异常，但不能单凭它断定每次轨迹跳变都由同一原因引起。

驱动原本在未同步模式下用主机 `high_resolution_clock` 给每包打时间戳、
控制出帧。MID-360 的无同步数据包本来就有纳秒级设备开机时间；
现在 `patches/livox_driver2_monotonic_clock.patch` 把第一包设备时间锚定到
Unix 时间，之后 LiDAR/IMU 共用同一设备时钟差值，并把出帧计时改为
`steady_clock`。PTP/GPS 同步模式仍使用原始设备时间戳。

FAST-LIO 原先 LiDAR 时间倒退时只清点云队列、没清对应时间队列；
IMU 时间倒退时也误报为 “lidar loop back”。
`patches/fast_lio_timestamp_guard.patch` 补上时间队列清理并重置待处理标志，
日志现明确写出是 LiDAR 还是 IMU 倒退及秒数。
`tools/setup_mid360_dependencies.bash` 在全新安装时自动应用两个补丁。
真实参数 YAML、外参、累计地图模式和两个 launch 入口均未改。

已重编 `livox_ros_driver2` 与 `fast_lio`，补丁可在各自固定提交上应用；
尚未连接真实雷达复测。下次接雷达后先静置初始化、再缓慢移动，观察
是否仍有 `timestamp moved backward` 和轨迹跳变。若还有跳变但没有
时间倒退，再检查网络丢包、IMU 数值、手持运动和几何退化；此修复不等于
证明整房地图达到 1 cm 精度。雷达重启或中途切换 PTP/GPS 模式后应重启建图。

## 2026-09-27：WSL 系统时钟修复（与上述旧包区分）

后续诊断包 `~/go2_bags/tilt_diagnostic_20260926_131440/` 是驱动补丁
生效后的另一组数据：按消息写入顺序，LiDAR/IMU 消息头时间戳没有倒退，
但 WSL 系统时间及 bag 接收时间仍会回拨。旧包的“IMU 消息时间戳倒退”
不能直接套用到这组数据，也不能据此判定当前雷达设备时钟再次倒退。

本机 WSL Ubuntu-22.04 已停用额外的 `systemd-timesyncd`，避免与 WSL
从 Windows 接收的隐式校时同时调整系统时钟；停用后 180 秒未再测到
大于 0.1 秒的回拨。详细证据、命令、恢复方法及 Windows 宿主机仍有
约 1.7 秒绝对时间偏差的注意事项见
[WSL ROS 环境与时钟记录](docs/WSL_ROS_ENVIRONMENT.md)。
这是本机系统设置，不会随 Git 仓库部署到其他电脑；原生 Ubuntu 上不要
照搬停用 NTP 服务。点云倾斜和运动定位仍需独立验证。
