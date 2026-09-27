# 导航参数修正与验证（2026-09-17）

## 修改

- 扫描配置节点键改为 `mid360_cloud_to_scan`，并由 launch 显式覆盖 `use_sim_time`。
- DWB、恢复旋转、速度平滑器上限降到 CHAMP 已有的 0.5 rad/s；未修改 PID 数值。
- 导航参数中删除无效 AMCL 段。
- 定位不再包含 `mapping.launch.py`，改为仿真 + 独立 `odometry.launch.py`。
- 里程计模式关闭 `publish.map_en`、路径发布和 PCD 保存，保存路径设为空；保留注册点云供 ICP 使用。
- FAST-LIO 仍维护内部局部地图估计里程计，不是完全不维护地图的固定地图定位算法。
- 地图默认路径不变；启动时打印参考 PCD 和二维地图路径，避免静默切换已验证地图。
- 删除仿真入口及 CHAMP Gazebo Python launch 中未使用的 `ros_control_file` 参数。
  PID 仍由 `go2_description/xacro/gazebo.xacro` 指定的配置加载。
- README 明确区分地图初猜和 Gazebo 出生位姿、地图分发副本和运行时地图。

## 已验证

- `go2_mid360_sim`、`go2_fastlio_localization`、`go2_nav_bringup` 编译通过。
- 顶层导航 launch 参数解析通过，不再暴露建图的 `save_pcd` / `map_output` 参数。
- 在独立 ROS_DOMAIN_ID=76 下实际运行扫描转换节点和里程计节点，读取参数服务：
  - 扫描：use_sim_time=true，target_frame=base_link，min_height=-0.15，max_height=0.5。
  - 里程计：pcd_save.pcd_save_en=false，publish.map_en=false，map_file_path 为空。
  - 里程计：publish.scan_publish_en 和 publish.scan_bodyframe_pub_en 均为 true。
- 测试节点已退出，未发送速度或导航目标，未改写参考地图。

## 2026-09-19 补充验收与未解决事项

- 三维 RViz 显示实验已于 2026-09-19 按用户要求撤回：恢复原始二维俯视、
  LaserScan、二维地图和 TF 箭头，删除本轮新增的两个三维点云显示项。
  仅撤回显示设置，不撤回导航启动链与扫描参数修正；显示变化不会修复避障数据。
- 扫描截面改为 `mid360_link` 上方 0.02～1.50 m，range_min=0.30 m；
  scan_time=0.20 s，与仿真雷达 5 Hz 一致。此策略仅适合当前墙柱平地演示，
  会漏掉低于截面的障碍物，不是通用地面分割，也未验证真实雷达或坡地。
- 重新编译三个 Go2 功能包通过。在独立 ROS_DOMAIN_ID=78、无 GUI 仿真中，
  等待 controller_server、planner_server、bt_navigator、velocity_smoother 均 active 后测试：
  - map (0.8, 0.0), yaw=0：返回 SUCCEEDED (4)。
  - map (0.8, 0.5), yaw=1.0：返回 SUCCEEDED (4)。
  - odom 起点约 (0.001, 0.014)，终点约 (0.788, 0.221)。
    odom 不等于 map，不能直接用这个终点计算地图目标误差；目前不是精度验收。
- ICP 日志中接受的 RMSE 从约 0.062 上升到约 0.29 m；
  匹配接受和 action 成功不能证明地图对齐精度可靠，需要更长路径与独立误差评估。
- `/scan` 有限距离数量范围 0～707：存在空扫描帧，需要进一步核对扫描模式、
  过滤截面与代价地图清除行为，尚未证明不会产生误清除或漏检。
- 地面回波独立核对本次获得 0 个时间匹配样本，不能作为排除地面点成功的证据。
- 启动日志有 `gazebo_ros2_control: Parameter 'hold_joints' has already been declared`；
  本次未阻止运动，但原因尚未定位，不能记录为“无报错”。
- 本轮测试进程已退出。尚未提交或推送；用户要求“没有问题再更新 GitHub”，
  因以上事项未闭环，暂不满足发布条件。

## 前一轮验收范围（历史）

没有在本轮重新启动完整 Gazebo 场景并执行导航目标。扫描高度过滤此前未生效，
修复后障碍物观测可能与旧运行不同；需在 RViz 检查地面/机身是否被过滤、墙与柱是否保留，
再验证一个短距离目标和一次转弯。不要仅凭编译通过就判断导航性能不变。
