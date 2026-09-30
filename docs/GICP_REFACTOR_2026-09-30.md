# GICP 执行模型与原生 Ubuntu 22.04 迁移记录

日期：2026-09-30。基线：项目 HEAD `0c3d2e2` 加工作区已有的日志、PCD 转换工具和文档改动。保留这些改动；本次修改实机 `go2_real_localization`，没有修改 Livox/FAST-LIO 依赖源码、实机建图参数或原始 PCD。

## 七项审查结论与处理

| 问题 | 结论及实现 |
| --- | --- |
| 同步 GICP 阻塞 ROS | 确认。输入、ROS 定时器和状态留在 executor；转换/滤波/配准/评分放独立 worker。只交接不可变输入快照和结果，不直接换多线程 executor。任务槽有界，不排队积压。 |
| 零时间戳首帧 | 确认。输入以显式 optional 时间及单调序号识别，0 是有效戳。新初值允许重新处理同一帧。 |
| valid 早于状态提交 | 确认。先更新 map_T_odom、调用 TF 发布，再发布配准位姿/点云与 valid。DDS 跨话题接收顺序仍不保证；下游必须确认对应 TF 可查询。 |
| wall timer / ROS 年龄混用 | 确认。配准调度、TF、年龄和 TTL 统一 ROS time；steady clock 仅计耗时。暂停时逻辑时间冻结，前跳按时效拒绝，后跳取消旧任务并要求新初值。提交时再次检查负年龄和过期。 |
| 残差不能证明唯一定位 | 确认。增加点到平面几何可观性检查，输出比例和约束数，拒绝弱约束平面/走廊。重复但结构丰富的房间仍需独立真值或全局检索；没有宣称解决全局歧义。 |
| source/target ROI 不一致 | 确认。source 限于 IMU 周围 local_radius；target 围绕预测位置，另加允许平移量和对应距离，覆盖边界附近合法对应点。内点率分母为 ROI 后的 source。 |
| 参数动态设置假生效 | 确认。启动配置全部 read_only，use_sim_time 也拒绝运行时切换。改 YAML/launch 后重启。 |

## 结构及线程所有权

`include/go2_real_localization/config.hpp`：配置结构、有限值/范围验证。

`registration_pipeline.hpp` + `src/registration_pipeline.cpp`：固定 PCD 加载、过滤、ROI、FastGICP、残差和退化检查；不使用 ROS Publisher、ROS 时间或节点 valid。

`localization_state.hpp` + `src/localization_state.cpp`：初值、输入序号、generation、map_T_odom、时效及恢复；不处理点云。所有方法由 ROS executor 调用。

`gicp_localizer_node.hpp` + `src/gicp_localizer_node.cpp`：ExactTime 输入、新初值、快照提交、结果接收、TF 和消息发布；`src/main.cpp` 保持单线程 spin。

`registration_worker.hpp`：唯一工作线程，mutex/condition_variable 只保护任务和结果交接。忙碌期间新输入替换节点最新输入；新初值/时间重置提升 generation，旧结果不能提交。退出会等待正在执行的有限迭代任务结束，不强行杀线程。

矩阵统一用 `map_T_body_guess`、`odom_T_body`、`map_T_body`、`map_T_odom`，其中 `A_T_B` 将 B 中的点转换到 A。

## 参数和兼容性

保留原来的话题、frame、可执行名称、launch 参数、10 cm 体素、1 Hz 配准以及原有质量门限。新增 `min_observability_ratio=0.001`。

退化检查使用内点附近 20 个目标近邻的 PCA 法向，只接受最小/中间特征值比不超过 0.05 的局部平面；旋转雅可比按有效点距 IMU 的 RMS 距离（至少 1 m）缩放，构建 6×6 几何信息矩阵。最小/最大特征值比低于门限，或平面约束数量少于 min_points，输出 DEGENERATE。此量不是位姿协方差，也不是全局匹配唯一性的证明；实际噪声和遮挡下须复测。显式设为 0 可禁用这道门。

已有 ACCEPTED/REJECTED 点数、残差、耗时日志保留，新增 observability_ratio/observability_points。source_points 现在包含 ROI 裁剪，不能与旧版远距离全扫描点数直接比较。

仅 `bringup.launch.py` 关闭 FAST-LIO 累计输出；将单独 localization.launch.py 附加到已启动的建图进程上，并不会关闭原进程的累计地图。

## 验证

测试源：

- `test/test_localization_core.cpp`：零戳、新初值重处理、旧结果丢弃、负/过期年龄、失效/回退、位姿组合、时钟冻结、参数有限值、工作线程有界交接、ROI、平面/走廊退化。
- `test/synthetic_check.py`：保留非零初猜位姿/TF 组合、无关扫描拒绝、恢复、断流失效。
- `test/runtime_check.py`：独立话题命名空间与重映射 /clock，验证零戳输入、暂停、前跳过期、后跳重新初始化和只读参数。

执行结果：ROS 2 Humble Release 构建通过；核心 gtest、原合成位姿/TF/拒绝恢复测试、ROS 时钟/只读参数运行测试通过。合成非零初猜测试平移差约 0.000062 m、yaw 差约 0.000004 rad，仅验证同一人工几何上的计算关系，不能当真实定位精度。新增提交前 TTL 检查覆盖 transform_timeout 小于 max_input_age 的配置，避免刚接受就已过期。

最初退化门过松时合成走廊仍被接受；使用更严格的平面邻域和 0.001 特征值比后，平面/走廊拒绝、完整房间接受的回归通过。最初运行测试也观察到 Bool 的回调先于 Pose 回调，验证了跨话题 DDS 接收顺序不能作为一致性契约，测试等待完整消息集合。

以上均为合成/接口验证，不代替纯 Ubuntu 电脑上的真实雷达移动验收。没有启动硬件节点、Gazebo、Nav2 或电控；所有测试进程均退出。代码和文档尚未提交/推送 Git。

## 在原生 Ubuntu 22.04 构建

目标电脑安装 ROS 2 Humble，复制/克隆源码与配套地图；重新构建，不复制 WSL 的 build/install。完整依赖安装使用仓库现有 setup 脚本，若依赖已就绪，本包增量构建：

```bash
cd ~/go2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rosdep install --from-paths src/unitree-go2-ros2/go2_real_localization --ignore-src -r -y
colcon build --packages-select go2_real_localization --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select go2_real_localization --event-handlers console_direct+
ros2 launch go2_real_localization bringup.launch.py map_path:=/绝对路径/房间地图.pcd
```

原生 Ubuntu 保留正常校时服务，不照搬 WSL 停用 systemd-timesyncd 的措施。雷达 JSON 的主机地址与实际有线网卡匹配；接口名和路由以该电脑为准。真实雷达 use_sim_time=false；仅回放使用 true 并发布 /clock。

## 仍未关闭的原审查条目

FAST-LIO 的 F01–F05 本轮未更改。G02 初值时间传播仍要求首次接受前静止；G04 节点退出后下游仍需独立接收超时；G05 ExactTime 保持同戳契约；G06 未加入全局地点识别。局部退化检测不能代替这些功能。
