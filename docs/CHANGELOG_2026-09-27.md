# 2026-09-27 改动与交付核查

本记录以仓库 `main` 的 `53df1fd` 为基线，说明此后本轮开发的改动。
它是变更与验证边界清单，不表示真实机器狗导航已验收。源代码与项目文档
在本仓库；`~/go2_maps`、`~/go2_bags`、WSL/Windows 系统设置和工作空间
中的第三方源码不在 Git 提交中。新机器应按安装脚本获取固定版本依赖，
不能只复制本机的 `install/`。

## 当前工作区主要改动

| 范围 | 本次状态与主要入口 |
| --- | --- |
| 仿真/导航 | `go2_mid360_sim` 的点云转 `/scan` 节点名与 YAML 匹配，扫描截面改为雷达坐标系上方 0.02～1.50 m；`champ_gazebo` 不再暴露未被插件读取的 PID 路径参数。`go2_nav_bringup` 去掉未启动的 AMCL 参数，旋转速度上限统一为 0.5 rad/s，启动时打印地图路径，并明确导航不保存 FAST-LIO 地图。仿真建图入口默认关闭 `/map_save` 写盘开关，默认输出目标改为独立的 `mapping_session.pcd`，避免误覆盖参考地图；实际保存仍用 `save_live_map`。见 [导航参数审计](NAVIGATION_PARAMETER_AUDIT.md)。仿真仍使用旧 ICP 链，尚未对新 FAST-LIO 依赖版本做完整 GUI/目标点回归。 |
| 真实雷达建图 | `go2_fastlio_localization` 新增真实驱动、FAST-LIO、组合建图入口与唯一的真实 YAML/JSON/RViz 配置；`save_live_map` 在运行中保存累计 `/Laser_map`。依赖固定到 Xju FAST_LIO，安装时应用手持近点/IMU 初始化、LiDAR/IMU 设备时钟与时间戳保护补丁。旧 `fast_lio_ros2.patch` 已删除；`fast_lio_local_map.patch` 仅保留为历史实验，不在当前安装脚本应用，且不能与现行手持补丁直接叠加。详见 [真实雷达 README](../README_REAL_MID360.md)。 |
| 真实地图 GICP 定位 | 新增 `go2_real_localization`，以固定版本 CPU `fast_gicp` 对已有 PCD 配准，输出 `map→odom`、质量状态和有效性。GICP 体素当前 YAML 为 0.10 m；平移步长收敛阈值恢复到固定上游的 0.0005 m，避免 0.000001 m 导致合成测试持续 `NOT_CONVERGED`。其余内点率、RMSE、修正量和时间门限仍生效。只做定位，不启动 Nav2 或机体控制；详见 [GICP 包说明](../go2_real_localization/README.md)。 |
| 安装与 Docker | `tools/setup_mid360_dependencies.bash` 现在在 `rosdep` 和全工作空间构建前调用固定版本 `fast_gicp` 获取脚本；否则全新安装和 Docker 都会缺少 `fast_gicp`。Dockerfile 增加对 `go2_real_localization` 的包发现检查。尚未执行全新 clone 的端到端安装或 Docker 全镜像重建。 |
| WSL 本机环境 | 用户在 WSL Ubuntu-22.04 执行了 `sudo systemctl disable --now systemd-timesyncd.service`，停用与 Hyper-V 宿主机同步冲突的第二套 NTP。180 秒监测未见超过 0.1 秒的回拨。该服务状态不会随 Git 推送；原生 Ubuntu 不应照搬。证据、剩余 Windows 绝对时间偏差与恢复命令见 [WSL ROS 环境记录](WSL_ROS_ENVIRONMENT.md)。 |

## 本次检查结果

- `bash -n`：安装脚本与 `setup_fast_gicp.bash` 通过；`git diff --check` 通过。
- 固定版本 `fast_gicp` 在本机工作空间可被 `colcon list` 发现；
  `fast_gicp`、`go2_fastlio_localization`、`go2_real_localization` 构建通过。
- 仿真改动涉及的 `champ_gazebo`、`go2_mid360_sim`、`go2_nav_bringup`
  增量构建通过；构建成功不代表 Gazebo 与 RViz 实测成功。
- 真实建图、实机 GICP 和仿真导航三个 launch 的 `--show-args` 检查通过；
  相关 Python 文件语法编译通过。这些检查未真正启动图形界面或雷达。
- GICP 合成回归在当前 0.10 m 配置下通过：位姿求解、`map→odom`
  组合、无关扫描拒绝、有效扫描恢复、输入过期失效。
  合成误差数值不能用作真实雷达厘米级精度声明。
- `rosdep install --simulate --from-paths src --ignore-src -r -y` 返回成功，
  但本机 rosdep 数据库仍对 `ament_python` 给出无规则警告；Humble
  工作空间实际可构建。未将此警告误写为全新环境已完整通过。

## 发布时必须保留的边界

1. 当前真实雷达已有 PCD 上的 GICP **未完成运动中定位、点云倾斜复现及
   与电控联动验收**；WSL 时钟回拨修复只排除一个已观测的问题。
2. `go2_real_localization/tools/pcd_to_nav2_map.py` 在裁剪框内把没有障碍
   命中的格子设为可通行，并非射线验证过的自由空间。输出仅是候选地图，
   未人工核查与规划安全验证前不可用于实机自主导航。
3. 当前 Xju FAST-LIO 检出不能依赖 Ctrl+C 自动保存 PCD；须运行中执行
   `ros2 run go2_fastlio_localization save_live_map`，确认 `SAVED` 后再停止。
4. GitHub 仓库不包含 `~/go2_maps` 中的真实 PCD、`~/go2_bags` 中的 rosbag、
   WSL systemd 状态，也不包含 `src/FAST_LIO`、`src/livox_ros_driver2`
   与 `src/fast_gicp` 的源码副本。安装脚本固定版本并重建依赖，真实地图
   若要分享，需要单独确认数据内容、体积和授权后提供。
5. Docker 镜像构建、全新 Ubuntu 部署、仿真 Nav2 Goal 与实机运动定位
   在本次提交前均未重新跑端到端验收；README 不应把这些写成本轮通过项。
