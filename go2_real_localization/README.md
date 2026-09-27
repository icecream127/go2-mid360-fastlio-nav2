# 实机 MID-360：已有 PCD 上的 GICP 定位

此包是实机专用入口，使用 koide3/fast_gicp 的 fast_gicp::FastGICP（CPU 多线程），不调用旧的 pcd_icp_localizer.py。不需要 CUDA。
不启动 Gazebo、Nav2 或电机控制。仿真继续使用 go2_mid360_sim / go2_nav_bringup 原入口；旧 ICP 文件保留兼容旧流程。

## 文件与职责

```
go2_real_localization/
  src/gicp_localizer.cpp          # 配准、质量检查、map→odom
  config/gicp.yaml               # 只放 GICP 参数
  launch/localization.launch.py # 只定位；已有 FAST-LIO 时使用
  launch/bringup.launch.py       # 实机驱动 + FAST-LIO + GICP
  rviz/localization.rviz         # 灰色地图、绿色对齐扫描、机体位姿
  test/synthetic_check.py        # 无硬件坐标/质量门限回归测试
```

真实传感器配置继续唯一存放于 go2_fastlio_localization/config/mid360_real.json 和 fast_lio_mid360_real.yaml；本包引用它们，不复制、不使用仿真 profile。
启动参数 user_config_path / lio_params / params_file 分别覆盖驱动、里程计、GICP 的配置路径。

## 编译与启动

完整部署若已运行仓库根目录的 `tools/setup_mid360_dependencies.bash`，
其中会在 `rosdep` 和工作空间构建之前准备固定版本的 `fast_gicp`。
以下命令也适用于现有工作空间只补装这两个包；重复运行 helper 会检查版本，
不会覆盖不匹配的已有源码。

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
bash src/unitree-go2-ros2/go2_real_localization/tools/setup_fast_gicp.bash
source install/setup.bash
colcon build --packages-select fast_gicp go2_real_localization --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=OFF -DBUILD_apps=OFF -DBUILD_test=OFF
source install/setup.bash
ros2 launch go2_real_localization bringup.launch.py \
  map_path:=/home/ice/go2_maps/real_room_20260923_231114_030542.pcd
```

地图路径必须明确传入，没有隐式选择“最新地图”。换房间时换成对应 PCD。
fast_gicp 依赖位于工作区 src/fast_gicp，版本固定为 0e7ec1441c99f7be453db2ea216d5de029387417；fast_gicp.repos 也记录了来源，便于迁移。
先关闭其他驱动、FAST-LIO、旧 ICP 定位启动终端，避免重复发布者。
如果驱动和 FAST-LIO 已在运行，只启动 `localization.launch.py map_path:=...`。
FAST-LIO 仍维护内部局部地图以估计运动，但新 bringup 关闭 publish.map_en 和 pcd_save.pcd_save_en，不累计发布或保存新的房间地图。

雷达保持静止完成 IMU 初始化。在 RViz 使用 2D Pose Estimate 指定地图上的大致机体位置和朝向。
该工具的 z=0；只有当前 IMU 高度接近建图初始化高度时才适合直接使用。
不同高度、倾斜姿态，应修改 gicp.yaml 的 initial_pose=[x,y,z,roll,pitch,yaw]（米/弧度），并启用 initial_pose_enabled，然后重新启动定位节点。
PCD 的 z=0 不一定是地面。这里的初始位姿描述当前 IMU 在建图坐标系中的位姿，不是雷达安装外参。
接收初始位姿到第一帧配准期间保持静止。初猜失败应重新给初值，不应无限放大质量门限。

```bash
ros2 topic echo /gicp/status std_msgs/msg/String --qos-durability volatile
ros2 topic echo /gicp/valid
ros2 run tf2_ros tf2_echo map livox_frame
```

## 接口与坐标

| 接口 | 类型/含义 |
|---|---|
| /cloud_registered_body | PointCloud2，已去畸变的 IMU 坐标点云 |
| /Odometry | Odometry，odom→livox_frame |
| /initialpose | PoseWithCovarianceStamped，map 中的 IMU 初猜；协方差暂不使用 |
| /gicp/map | PointCloud2，降采样已有地图，transient-local |
| /gicp/aligned_cloud | PointCloud2，最近一次通过检查的配准点云 |
| /gicp/pose | PoseStamped，扫描时间的 map→IMU 位姿 |
| /gicp/status | String，状态及 RMSE/内点比例/修正幅度 |
| /gicp/valid | Bool，最近配准是否通过且尚未过期 |

源码确认该 FAST-LIO fork 的 livox_frame 实际是 IMU 机体坐标系，不应仅凭名字当成光学测量原点。
两路输入使用 ExactTime 配对（上游均使用 lidar_end_time），避免拿错时刻里程计。
GICP 求 T_map_body；发布 T_map_odom = T_map_body * inverse(T_odom_body)。
TF 链：map --本包--> odom --FAST-LIO--> livox_frame。不要同时运行其他 map→odom 发布者。
上机器马后，须测量并接入机体与 IMU 外参；本阶段不伪造 base_link。

## 参数与失败处理

当前 YAML 的地图/扫描体素为 0.10 m，1 Hz 配准；这些是起步计算配置，
不是精度承诺。C++ 中的 0.15 m 仅是未加载 YAML 时的回退默认值。
local_radius 是围绕预测机体位置裁剪目标地图的半径。max_iterations / correspondence_distance 控制配准。
inlier_distance 是评估内点的距离门限；max_rmse 是内点最近邻距离的均方根（米），不是 PCL fitness 的平方距离。
min_inlier_ratio 与位姿修正门限同时检查，不能仅凭 hasConverged 判定定位可靠。
这些检查不保证排除重复走廊等几何歧义，也没有全局搜索或回环优化。
输入过期、时间回退、frame 不匹配或配准失败会报告失效；失败不发布新修正 TF。
TF 缓存与 RViz 可能仍显示旧结果，因此接运动控制前必须接入 /gicp/valid 和超时停车。
时间回退后需重新给初始位姿。FAST-LIO 重启也应一并重启本定位节点。
节点按最新扫描处理，不累积历史点云；fast_gicp 默认使用 4 个计算线程（num_threads），ROS 回调串行执行，需实测确认配准耗时满足输入时效。
当前 C++ 使用所固定 fast_gicp 版本的 0.0005 m 平移步长收敛阈值；
先前硬设 0.000001 m 时，0.10 m 体素配置的合成测试在 40 次迭代后持续
报 `NOT_CONVERGED`。恢复上游阈值后仍须通过内点率、RMSE、位姿修正量和
输入时效检查，不能把“算法收敛”直接等同于“定位正确”。

## 验收边界

先验证静止、平移、转弯、回到起点，检查灰色地图与绿色扫描的重合及状态。
合成测试只能验证数学关系、ROS 接口和拒绝逻辑；不能代替真实房间定位测试，不能据此宣称厘米级精度。

## 本次实现记录（2026-09-24）

- 新增独立实机包，仿真启动链与旧 ICP 未改动；使用 fast_gicp::FastGICP CPU 4 线程。
- 固定上游依赖版本，源码放 src/fast_gicp；无需修改上游源码。
- 新包与依赖已在 Ubuntu 22.04 / ROS 2 Humble 编译通过。
- 已加载 real_room_20260923_231114_030542.pcd，0.15m 降采样后为 11889 点。
- 合成集成测试通过：非零初猜配准、map→odom 组合、无重合扫描拒绝、有效扫描恢复、输入过期失效。
- 本机独立 ROS_DOMAIN_ID=84 未发现测试节点；默认环境下上述测试通过。测试命令使用默认环境，不应据此修改真实雷达的网络参数。
- 尚未完成真实雷达移动定位验收；未启动机器马、Nav2 或电控。

2026-09-27 在当前 0.10 m YAML 配置下重新构建并运行合成测试：
位姿与 `map→odom` 组合、错误扫描拒绝、有效扫描恢复和过期输入失效
均通过；这仍不是实机运动验收。

复测（先停实机定位，避免同名话题）：

```bash
source ~/go2_ws/install/setup.bash
unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY
python3 ~/go2_ws/src/unitree-go2-ros2/go2_real_localization/test/synthetic_check.py
```
