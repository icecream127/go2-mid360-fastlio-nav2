# 真实 MID-360 建图：本轮改动与排查手册

> 2026-09-22 更新：真实建图工具已整理为 ROS 2 包入口，当前启动/保存方式见
> [功能包 README](../go2_fastlio_localization/README.md)。新保存工具默认目录为
> `~/go2_maps/`。下文保留 9 月 21 日历史参数与旧操作，便于追溯。

更新：2026-09-21。范围：真实雷达接入、FAST-LIO 静止漂移、RViz 卡顿、保存地图及退出。
不是整个项目历史改动清单；Git 工作区里还有之前的仿真、导航改动，不应全部归因于本次排查。
本文记录当前实际文件与测试结果，不代表已完成实机导航或厘米级绝对精度验收。

## 1. 下次怎么用

环境：本机 WSL2 / Ubuntu 22.04 / ROS 2 Humble；工作空间 `/home/ice/go2_ws`。
将雷达固定在稳定支架上；确认有线网络连接，关闭旧的驱动、仿真和 FAST-LIO。

### 启动真实建图（按需另存，避免覆盖旧地图）

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_real_mid360_mapping.bash save_pcd:=false
```

等待 `IMU Initial Done`（正常约 10 秒，丢消息时可能更久），继续静止至少一分钟再缓慢移动。
该脚本启动真实驱动、FAST-LIO、RViz，不启动 Gazebo、Nav2 或机器狗控制。
无需窗口时追加 `show_rviz:=false`。

### 保存当前累计点云，不停止建图

另开 WSL 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 ~/go2_ws/real_mid360/save_live_map.py
```

脚本从 `/Laser_map` 读取一条累计地图，检查 XYZ，移除非有限点，保存二进制 PCD。
保留 XYZ 和存在时的 intensity，坐标系为 `camera_init`；不是只保存当前一帧扫描。
输出 `~/go2_ws/maps/real_room_年月日_时分秒.pcd`，独占创建，已有同名文件会报错而非覆盖。
最多等待 45 秒，无消息/空点云则报错。依赖当前建图仍运行且 `publish.map_en=true`。
关闭程序后此脚本不能从消失的内存恢复地图。PCD 不包含 rosbag 的 IMU/完整时间序列，也不是 Nav2 二维地图。

### 退出

看到 `SAVED` 和点数后，在启动建图的终端按 Ctrl+C，等待驱动、FAST-LIO、RViz 一起退出。
只关闭 RViz 不会停止驱动和建图。不要用 `wsl --shutdown` 代替正常保存退出。
不响应时先检查具体进程，勿盲目批量杀掉所有 ROS 进程：

```bash
ps -eo pid,ppid,args | grep -E 'real_mapping.launch|fastlio_mapping|mid360_real_driver|fast_lio_real_rviz' | grep -v grep
```

本次已保存并检查文件尺寸：

- `/home/ice/go2_ws/maps/real_room_20260921_220233.pcd`
- 1,822,855 点，29,165,841 字节，约 28 MiB，`camera_init` 坐标系。
- 未覆盖旧 `real_mid360_room.pcd`；保存后相关进程已关闭。
- 仅检查写入和尺寸，不代表已对整张地图几何质量做独立验收。

## 2. 真正生效的是哪个配置

```text
tools/run_real_mid360_mapping.bash
  └─ go2_fastlio_localization/launch/real_mapping.launch.py
       ├─ livox_ros_driver2 + config/mid360_real.json
       │    ├─ /livox/lidar : livox_ros_driver2/msg/CustomMsg
       │    └─ /livox/imu   : sensor_msgs/msg/Imu
       ├─ fast_lio/fastlio_mapping + config/fast_lio_mid360_real.yaml
       │    ├─ /Odometry : nav_msgs/msg/Odometry
       │    ├─ /cloud_registered : sensor_msgs/msg/PointCloud2
       │    └─ /Laser_map : sensor_msgs/msg/PointCloud2（累计点云）
       └─ rviz2 + config/real_mapping.rviz
```

上面的 config 路径均在 `go2_fastlio_localization/` 下。ROS launch 实际通过包的
install/share 目录加载它们，不应只看编辑器里某个同名 YAML 就判断配置生效。
`real_mapping.launch.py` 后面的参数覆盖 YAML 中的 `map_file_path` 与 `pcd_save.pcd_save_en`。

**本条启动链不读取 `src/FAST_LIO_ROS2/config/mid360.yaml`。**
FAST-LIO 依赖源是 `https://github.com/Ericsii/FAST_LIO_ROS2.git`，本机基础提交为
`2fffc570a25d0df172720bac034fbdb6a13d2162`，另有本地补丁。
这是社区 ROS 2 移植，不是 Livox 官方驱动；不能仅凭它不是原作者仓库就断言漂移原因。

## 3. 修改文件与参数对照

下面相对路径以 `/home/ice/go2_ws/src/unitree-go2-ros2/` 为根。
“之前”仅指本轮排查时的配置，不代表项目最初版本。新建文件没有可准确声称的旧值。

### 3.1 `go2_fastlio_localization/config/fast_lio_mid360_real.yaml`

| 参数 | 排查前 → 当前 | 目的/注意事项 |
| --- | --- | --- |
| `point_filter_num` | 3 → 1 | 减少按点数抽样，增加室内几何约束与运算量 |
| `max_iteration` | 3 → 5 | 增加滤波更新最大迭代次数，不是每次必跑 5 次 |
| `filter_size_surf` | 0.25 → 0.05 m | 保留更细的扫描几何 |
| `filter_size_map` | 0.25 → 0.05 m | 保留更细的地图几何 |
| `cube_side_length` | 200 → 1000 m | 避免不合适的局部窗口裁剪触发条件 |
| `mapping.extrinsic_est_en` | 当前 false；本轮早期已关闭在线估计 | 固定标称外参，不能等同于完成外参标定 |

局部窗口源码判断为距离边界 `<= 1.5 * det_range` 时触发移动。
当前 `det_range=100`，原窗口半宽为 100，却小于阈值 150，中心位置也满足触发条件。
这是确认的配置不匹配；它对误差的独立贡献没有用严格单变量实验完全分离。
1000 是局部空间范围，不代表立即分配一个填满点云的 1000 米立方体。

当前其他关键值（主要用于下次核对，不都是本轮新改）：

- `feature_extract_enable=false`。
- `common.lid_topic=/livox/lidar`，`imu_topic=/livox/imu`。
- `common.time_sync_en=false`，`time_offset_lidar_to_imu=0.0`；不要为了看起来同步随意填偏移。
- `preprocess.lidar_type=1`，`scan_line=4`，`blind=0.5`，`scan_rate=10`，`timestamp_unit=3`。
  当前走 CustomMsg 分支；不能只改通用 timestamp_unit 就假定改了该分支的逐点时间处理。
- `mapping.acc_cov=0.1`，`gyr_cov=0.1`，`b_acc_cov=b_gyr_cov=0.0001`。
- `mapping.fov_degree=360.0`，`det_range=100.0`。
- `mapping.extrinsic_T=[-0.011,-0.02329,0.04412]` 米，`extrinsic_R` 为单位阵。
- `publish.map_en=true`、`scan_publish_en=true`、`dense_publish_en=true`、`scan_bodyframe_pub_en=true`。
- YAML 中 `pcd_save.pcd_save_en=true`、`interval=-1`，但启动参数 `save_pcd:=false` 会覆盖为 false。

**0.05 m 体素不是“误差 5 cm”，更不是精度保证。细体素增加 CPU/内存负担，需在线测试。**

### 3.2 FAST-LIO 源码与可复现补丁

实际源码在仓库外：`/home/ice/go2_ws/src/FAST_LIO_ROS2/src/IMU_Processing.hpp`。

| 位置/符号 | 修改 | 说明 |
| --- | --- | --- |
| `MAX_INI_COUNT` | 10 → 2000 | 累计 IMU 样本，不是 2000 帧雷达；200 Hz 下约 10 秒 |
| `last_lidar_end_time_` 成员 | 无初始化 → `= 0.0` | 避免未初始化时间参与首轮计算 |
| `ImuProcess::Process` 初始化分支 | `last_imu_ = meas.imu.back()` 后增加 `last_lidar_end_time_ = meas.lidar_end_time` | 以初始化扫描结束时间作为后续预测起点 |

改动同步保存在本仓库 `patches/fast_lio_ros2.patch`。补丁中原有的
`laserMapping.cpp` 退出保存累计地图修改不是本次新增，需保留。
延长初始化是保守措施，不应把“原始计数是 10”直接判定为所有漂移的根因。
没有加入强制锁定位姿、用真值替代测量或把静止输出人为置零的逻辑。

此初始化计数是编译期全局常量，使用同一个 fast_lio 可执行文件的仿真也会受影响，
不是只对真实 launch 生效。本次未重新验证所有仿真启动链；后续可考虑改成可配置参数。

### 3.3 `go2_fastlio_localization/config/real_mapping.rviz`

| 配置 | 之前 → 当前 |
| --- | --- |
| `FAST-LIO Odometry / Keep` | 100 → 1 |
| `Registered Cloud / Decay Time` | 60 → 0 秒（只显示最新消息） |
| `Accumulated Map / Enabled` 和 `Value` | true → false |
| `Global Options / Fixed Frame` | 当前 `camera_init` |

这样减少渲染积累，不改变估计位姿，不清空 FAST-LIO 内存地图。
累计地图发布仍开着，所以建图很久之后内存/发布开销仍可能增长；本次未解决长期无界增长。
想看完整地图可临时勾选 Accumulated Map，但注意大点云开销。

### 3.4 真实驱动与启动文件

- `go2_fastlio_localization/config/mid360_real.json`：真实设备专用网络配置。
  当前雷达 `192.168.1.111`，主机各数据接收 IP `192.168.1.50`。
  雷达侧命令/推送/点云/IMU/日志端口为 56100/56200/56300/56400/56500；
  主机对应端口为 56101/56201/56301/56401/56501。换电脑须核对 IP，不能原样套用。
- `go2_fastlio_localization/launch/real_lidar.launch.py`：只启动真实驱动。
  `xfer_format` 默认 1（CustomMsg），可传 0（PointCloud2）；`publish_freq=10.0`，
  `use_sim_time=false`，`frame_id=livox_frame`，`data_src=0`，`multi_topic=0`。
- `go2_fastlio_localization/launch/real_mapping.launch.py`：驱动固定 `xfer_format=1`；
  加载真实 FAST-LIO YAML，RViz 延迟 4 秒启动（不是数据已就绪的保证）。
  参数 `show_rviz=true`、`save_pcd=true`；默认保存路径 `~/go2_ws/maps/real_mid360_room.pcd`。
- `tools/run_real_mid360_mapping.bash`：配置 ROS 环境、SDK 库路径、`QT_QPA_PLATFORM=xcb`、
  `ROS_DOMAIN_ID=0`、`ROS_LOCALHOST_ONLY=1`。新增 flock 锁与已有驱动/FAST-LIO 进程检查。
  当前硬编码 `/home/ice/go2_ws`，换用户名或工作空间时需要修改；不是通用安装脚本。

### 3.5 排查脚本、记录和仓库外文件

| 路径 | 用途/本轮状态 |
| --- | --- |
| `tools/check_fastlio_stationary.py` | 位姿采集 45 → 65 秒；增加数据时间覆盖范围、相对首帧最大位移和非绝对精度提示 |
| `tools/check_real_mid360_timing.py` | 本轮早期辅助检查 CustomMsg/IMU、点时间和数量；曾遇退出卡住，尚不是可靠验收入口 |
| `docs/MID360_REAL_BRINGUP.md` | 更新真实数据接收、配置、回放结果与尚未验收事项 |
| `~/go2_ws/real_mid360/benchmark_mapping.py` | 离线启动 FAST-LIO/回放/采集位姿；当前最后入口是两次 0.5 倍速试验，禁用地图发布与自动保存 |
| `~/go2_ws/real_mid360/save_live_map.py` | 本次新增累计地图另存脚本，见第 1 节 |
| `~/go2_ws/real_mid360/benchmark_results/` | 各组结果 JSON、逐帧位姿和日志 |
| `~/go2_ws/bags/mid360_stationary/` | 约 21.6 秒真实数据，约 216 帧点云与 4313 条 IMU |
| `~/go2_ws/real_mid360/run_raw.bash`、`raw.launch.py`、`raw.rviz`、`MID360_config.json` | 前序原始点云查看入口；本轮漂移修复未重新改造，不与建图同时运行 |

**`~/go2_ws/real_mid360`、`maps`、`bags` 不在当前 Git 仓库根目录内。**
在 `src/unitree-go2-ros2` 中运行 git add/push 不会上传这些文件。
换机器需要另行备份；室内地图/录包可能包含私人空间信息，不要无意公开。
本轮没有执行 Git 提交或推送，也没有覆盖用户既有地图。

## 4. 踩坑与排查顺序

| 现象 | 本次认识/正确处理 |
| --- | --- |
| 雷达未动，地图却移动 | 先确认单一驱动和单一 FAST-LIO，再查初始化、时间、外参和地图窗口；不能直接怪 WSL 或硬件 |
| 测试结果有时巨大、有时没有消息 | 曾有旧建图进程并行影响试验；混在同一域的数据不可作为算法结论。零采样不是零误差 |
| 配置改了没变化 | 核对实际 launch 加载的 share 配置及参数覆盖；不要误改依赖包默认 mid360.yaml |
| RViz PointCloud2 找不到 /livox/lidar | 建图输入是 CustomMsg；显示 `/cloud_registered` 或 `/Laser_map`。直接切驱动为 0 会破坏当前 FAST-LIO 输入链 |
| RViz 很卡/很多重复箭头 | 减小历史保留，默认关闭累计地图显示；这不能治疗算法漂移 |
| 原始点云能显示，SLAM 显示不对 | 原始数据 frame 通常 livox_frame；建图输出 camera_init。先读消息 header，再匹配 Fixed Frame，勿随意加零 TF 掩盖问题 |
| ROS CLI 卡住、不同终端互相看不到 | 核对 source、ROS_DOMAIN_ID、ROS_LOCALHOST_ONLY、RMW 和 daemon；反复 daemon start 不能证明根因已解决 |
| ping 成功/驱动建了发布者，但收不到数据 | 只说明部分链路通；还需实收点云和 IMU，核对主机接收 IP/UDP、域与 QoS |
| save_pcd:=false 后 map_save 返回失败 | 源码服务检查 pcd_save_en；不能假定 ros2 param set 会更新启动时读取的 C++ 变量。此次改用累计点云订阅保存 |
| 关闭 RViz 后脚本提示已有进程 | 驱动与 FAST-LIO 仍在；从原 launch 终端正常结束，不删除锁文件强行绕过 |
| Init Done 前暂时 No point | 初始化阶段可能出现，不能仅凭这一行断言无雷达；持续出现则查实际输入与初始化进度 |

原始 CustomMsg 中点时间不一定按数组顺序递增，源码去畸变前会排序；不能仅凭
offset_time 有回退就宣称时间戳损坏。扫描结束时间推算与同步的运动场景验证仍需进一步做。
早期真实驱动退出有过 exit -7 / -6、内存释放错误，尚未证明所有退出兼容性问题已根治。

## 5. 验证结果与不能承诺的事情

同一短录包，初始化约占前 10 秒。最大漂移定义为各位姿相对第一个有效位姿的距离最大值，
不是所有点对最大距离，也不是相对真实坐标的误差。

| 试验 | 有效时间 | 首末位移 | 最大相对位移 | 备注 |
| --- | --- | --- | --- | --- |
| 原窗口/降采样配置 | 约 11.2 秒 | 83.10 mm | 149.08 mm | 作为本次基线，不是所有历史版本 |
| 0.05 m 密度与新窗口，时间修复前 | 约 11.2 秒 | 3.25 mm | 5.01 mm | 一次有效回放 |
| 时间修复后，半速重复 1 | 约 11.2 秒，113 帧 | 3.28 mm | 4.89 mm | 首末旋转约 0.150° |
| 时间修复后，半速重复 2 | 约 11.2 秒，113 帧 | 3.28 mm | 4.89 mm | 同一数据重复，不是独立实测 |

最终结果文件：`~/go2_ws/real_mid360/benchmark_results/verified_half_rate_0.json` 和 `_1.json`。
测试按传感器时间计算持续时间。离线评测禁用了累计地图发布/PCD 保存，且采用半速，
负载比当前完整在线链小；不能宣称实时性能已验证。部分全速测试为零采样或只覆盖后段，未当作通过。

已做：fast_lio 和 go2_fastlio_localization 编译成功、Python 语法检查、启动参数展示检查、
源码补丁反向匹配检查、两次完整离线结果、累计点云实际保存、相关进程实际退出。

未完成：新配置在线 60 秒/5 分钟静止测量、10 Hz 实时处理负载、移动/转弯/回起点测试、
独立基准测距、长期大地图内存测试、全部仿真链回归。当前 FAST-LIO 链没有额外闭环优化。
**没有“整张地图绝对误差 ≤1 cm”的证据。** 后续需独立尺量/控制点和运动重复性测试，
不要把静止输出稳定、体素大小或重复播放同一 bag 当成绝对精度证明。

## 6. 修改后怎么编译和检查

源码或包内容修改后：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_lio go2_fastlio_localization --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch go2_fastlio_localization real_mapping.launch.py --show-args
```

只改 YAML/RViz 且确实是 symlink-install 链接时通常重启即可；不确定时重新编译配置包。
运行中的节点不会因为编辑文件自动重新加载。源码改动必须重编译。
FAST-LIO 补丁已经应用，不要重复 git apply；检查当前源码是否包含补丁：

```bash
cd ~/go2_ws/src/FAST_LIO_ROS2
git apply --reverse --check ../unitree-go2-ros2/patches/fast_lio_ros2.patch
```

这是只检查，不撤回修改。若报错先查看源码差异，勿用 reset --hard 覆盖其他改动。

启动稳定后，另开终端做静止检查（雷达全程固定）：

```bash
source /opt/ros/humble/setup.bash
source ~/go2_ws/install/setup.bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
python3 ~/go2_ws/src/unitree-go2-ros2/tools/check_fastlio_stationary.py
```

约 65 秒后打印有效数据跨度与漂移。先看是否有足够样本与连续覆盖，再解读毫米数值。
未完成独立测量之前，应将当前状态称为“真实 MID-360 建图链已跑通，静止漂移已改善，继续验收”。
