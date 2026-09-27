# 真实 MID-360 接入记录

本轮完整参数对照、保存脚本、踩坑与验收边界见
[真实 MID-360 建图排查 README](README_REAL_MID360_TROUBLESHOOTING.md)。
下文为阶段记录；后续保存地图和关闭程序的结果以该 README 为准。

## 本机配置（2026-09-20）

- Windows 有线网卡和 WSL 镜像网卡：192.168.1.50/24。
- 实际应答雷达：192.168.1.111，WSL ping 3 次全部成功。
- 独立配置：go2_fastlio_localization/config/mid360_real.json。
- 独立启动：go2_fastlio_localization/launch/real_lidar.launch.py。
- use_sim_time=false；默认 CustomMsg，10 Hz；frame_id=livox_frame。
- 不启动 Gazebo、SLAM、Nav2 或机器人控制。不要与仿真或另一驱动同时启动。

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_fastlio_localization real_lidar.launch.py
```

仅需 RViz 通用点云时可加 xfer_format:=0，输出类型改变后 FAST-LIO 的
CustomMsg 输入路径不能直接订阅该话题；不要同时启动两个驱动。

## 早期测试记录（已由后续数据接收测试更新）

驱动 1.2.6 日志确认：配置解析成功、设备工作模式设置成功、数据格式设置成功、
启用 IMU 成功，且创建了点云与 IMU 发布者。它确认应答设备地址为 192.168.1.111。
但是三个短时跨进程订阅测试均收到 0 条点云与 IMU 消息（含独立 ROS 域和 UDP-only 对照）。
这是早期测试结果，并非当前状态；后续已接收到点云和 IMU，见下文。

驱动关闭时出现过 exit -7 和 munmap_chunk(): invalid pointer / exit -6，
还需排查驱动/依赖兼容性和 ROS 2 数据传递。未更改防火墙、网卡或系统网络设置。
本轮测试驱动已关闭；配置与编译完成不等于数据链验收完成。

## 2026-09-21：真实数据与静止漂移排查

已录得约 21.6 秒真实数据：点云约 10 Hz、IMU 约 200 Hz。FAST-LIO 使用
Livox CustomMsg（xfer_format=1），不是仅供通用 RViz 查看用的 PointCloud2。
实际生效的参数是 `go2_fastlio_localization/config/fast_lio_mid360_real.yaml`，
并非依赖包里的 `FAST_LIO_ROS2/config/mid360.yaml`。

本轮改动：

- 真实建图局部地图窗口从 200 改为 1000。原配置半宽 100，小于源码移动阈值
  `1.5 * det_range = 150`，中心位置也会触发窗口移动/裁剪。
- 室内试验配置使用 0.05 m 体素、point_filter_num=1、5 次最大迭代，保留更多几何约束。
  这是精度优先配置，不能把 5 cm 体素理解为 5 cm 测量误差；需验证实时算力。
- 保持标称雷达/IMU 外参固定；初始化累计 2000 个 IMU 样本，约 10 秒。
  延长初始化是保守措施，并未单独证明原始初始化长度就是漂移根因。
- 修复 IMU 首次预测中上一帧结束时间未初始化的问题。源码变更保存在
  `patches/fast_lio_ros2.patch`，并已重新编译 fast_lio。
- RViz 当前点云不再保留 60 秒历史，轨迹箭头只保留 1 个；累计地图默认不显示。
  这只是显示减负，不冻结位姿，不关闭 PCD 保存。
- 启动脚本增加重复启动检查，避免多个驱动/FAST-LIO 混发数据。

短数据回放中，原参数首次至末次位移约 8.31 cm，最大偏移约 14.91 cm；
窗口和密度调整后的 0.05 m 配置，在时间初始化修复前的一次约 11.2 秒有效输出中，
末端偏移约 3.25 mm，最大偏移约 5.01 mm。这是有限的静止相对漂移结果，
不是雷达绝对测距精度，也不是整个房间的 SLAM 绝对精度认证。
部分回放收到零个位姿或只覆盖后半段，这些不能作为完整验收数据。

时间初始化修复编译后，半速回放重复两次均得到 113 个位姿，覆盖约 11.2 秒
传感器时间：末端偏移 3.28 mm，最大偏移 4.89 mm，首末姿态差约 0.150 度。
使用半速是为完整处理离线数据，不能据此宣称真实在线处理性能已通过。
原始数据及逐帧结果保留在本机 `~/go2_ws/bags/mid360_stationary` 和
`~/go2_ws/real_mid360/benchmark_results/verified_half_rate_*.json`。

### 重新运行

先关闭旧驱动/建图启动终端。将雷达固定在稳定支架上，不要手持初始化。

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_real_mid360_mapping.bash
```

等待 `IMU Initial Done`，先保持静止观察至少 60 秒，再缓慢移动，避免猛转。
默认输出 `~/go2_ws/maps/real_mid360_room.pcd`；已有同名地图应先备份，
也可用 `map_output:=/home/ice/go2_ws/maps/room_new.pcd` 指定新文件。

### 尚未通过的验收

- 新配置真实在线持续 60 秒和 5 分钟的静止漂移，以及实时处理是否跟得上 10 Hz。
- 运动、转弯、回到起点的重复性；该 FAST-LIO 链本身没有额外闭环优化。
- 用独立测量基准对比墙距/控制点，验证绝对误差；不能用自身输出证明 1 cm 准确度。

本轮不自动启动硬件或 RViz，不替用户覆盖地图，不提交或推送 GitHub。
