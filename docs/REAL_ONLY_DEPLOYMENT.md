# 纯实机部署：MID-360 + FAST-LIO + GICP

2026-10-13。适用于 Ubuntu 22.04 / ROS 2 Humble，目标包括 Jetson ARM64。
此入口不安装 Gazebo、Livox 仿真插件、CHAMP 或 Nav2；使用 CPU fast_gicp，不要求 CUDA。
脚本仍使用工作空间的 `build/`、`install/`、`log/`，日常继续 `source install/setup.bash`。

## 1. 新电脑

先安装 Ubuntu 22.04 对应的 ROS 2 Humble，ROS 软件源必须可用。
已有 ros-base 可以使用本脚本补齐 RViz、rosbag 和其他实机依赖；脚本不安装 ROS 本身。

```bash
sudo apt update
sudo apt install -y git
mkdir -p ~/go2_ws/src
cd ~/go2_ws/src
git clone https://github.com/icecream127/go2-mid360-fastlio-nav2.git unitree-go2-ros2
cd unitree-go2-ros2
bash tools/setup_real_mid360_dependencies.bash --check
bash tools/setup_real_mid360_dependencies.bash
```

已有仓库不重复 clone；更新源码后执行脚本即可。有本地改动时先检查 `git status`，
不要为了更新而强制覆盖。不要使用 `sudo bash` 运行整个脚本，以免源码和构建产物归 root 所有。
建议使用新终端，避免加载其他工作空间后再安装。

Jetson 内存紧张可降低构建并发：

```bash
GO2_BUILD_JOBS=1 bash tools/setup_real_mid360_dependencies.bash
```

默认每次构建一个包，每包最多 2 个编译任务。出现编译器被 Killed 时先检查内存，
不能把中断构建当安装成功。脚本最后应显示：

```text
Real MID-360 dependencies and workspace build completed.
```

## 2. 下载与构建范围

| 源码目录 | 固定提交 / 职责 |
| --- | --- |
| `src/Livox-SDK2` | `f5d9375f84efe2b15bc0a052d3e18482ed13adf4`，先安装 SDK 到工作空间 |
| `src/livox_ros_driver2` | `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`，真实驱动 |
| `src/FAST_LIO` | Xju `440a8e3e909023b7cb084e99b0c5070e3baebe86` |
| `src/fast_gicp` | `0e7ec1441c99f7be453db2ea216d5de029387417`，CPU 配准 |

应用现行 Livox 设备时间、FAST-LIO 手持和回拨保护补丁。
ROS 构建范围严格限定为 `livox_ros_driver2`、`fast_lio`、`fast_gicp`、
`go2_fastlio_localization`、`go2_real_localization` 五个包。SDK 单独构建。

为兼容现有入口，`go2_fastlio_localization` 暂时仍同时分发实机与仿真 launch，
包清单中的 `go2_mid360_sim` 依赖也保留。纯实机脚本通过明确源码路径和
`rosdep --skip-keys go2_mid360_sim` 排除这条仅仿真启动使用的依赖；
这不是已完成全包结构拆分。`ament_python` 构建类型也显式跳过 rosdep 解析，
对应 colcon Python 构建工具已由 apt 安装。
不要在这个部署流程中改用无范围的 `rosdep --from-paths src` 或直接全量 `colcon build`，
否则会重新遍历仿真包。

## 3. 之前装了一部分，怎么处理

- **之前 apt 报找不到 Gazebo 包：**不用重装 ROS，也不用卸载已经成功安装的系统依赖，直接用新脚本。
- **已经克隆了整个项目或仿真依赖：**保留；脚本不扫描构建这些目录。
- **已经有 build/install/log：**保留，新脚本重新配置并增量构建指定实机包；不清空工作空间。
  老仿真包仍可能被 `ros2 pkg list` 找到，但 source 环境不会自动启动它们。
- **存在固定版本的依赖源码：**原地复用，保留修改；必需补丁已存在则跳过，不重复应用。
  本地修改若与补丁冲突，脚本停止，不强制 reset。
- **依赖版本不一致或上次下载不完整：**脚本停止并打印路径，不删除、不切换该目录。
  检查后可将明确有问题的目录备份到 src 外，再重跑。例如脚本明确报告 FAST_LIO 版本错误时：

```bash
mkdir -p ~/go2_ws/dependency_backups
mv ~/go2_ws/src/FAST_LIO \
  ~/go2_ws/dependency_backups/FAST_LIO_before_real_$(date +%Y%m%d_%H%M%S)
bash ~/go2_ws/src/unitree-go2-ros2/tools/setup_real_mid360_dependencies.bash
```

这只是指定目录的手动备份示例，不需要每次执行。备份保留原修改，但不会自动合并到重新下载的依赖。
失败的新下载保存在 `dependency_downloads/`，不放进 src，避免 colcon 发现半成品。
脚本不删除或覆盖真实 PCD、bag、JSON 或 YAML。

## 4. 配置并启动

配置项目中的 `go2_fastlio_localization/config/mid360_real.json`，使主机 IP 与接雷达的有线网卡一致。
当前示例为主机 192.168.1.50、雷达 192.168.1.111：

```bash
ip -br address
ip route get 192.168.1.111
```

不要照抄其他电脑的 eth3，也不要照搬 WSL 停用校时服务的设置。
真实地图需要另行复制到 `~/go2_maps/`；安装脚本不复制仿真地图，也不包含旧电脑的真实地图。

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch go2_real_localization bringup.launch.py \
  map_path:=$HOME/go2_maps/real_room_20260926_110555_607459.pcd
```

将地图名换成实际文件。无桌面加 `show_rviz:=false`，并通过初值配置或其他 ROS 终端提供初始位姿。
只建图时用 `ros2 launch go2_fastlio_localization real_mapping.launch.py`，两个总入口不要同时运行。

## 5. 验证边界

脚本需要在 Jetson 上实际执行并验证编译和雷达运行；x86 本机语法/依赖范围检查不等于 ARM64 实测通过。
本次没有重装本机依赖，也没有移动现有依赖或启动雷达。移植后先验证话题、时间戳、静止定位，再做移动测试。
