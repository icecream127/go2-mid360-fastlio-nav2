# 2026-09-23：恢复 FAST-LIO 局部地图发布

> **历史实验记录，非当前部署状态。** 后续为覆盖整间房而恢复了累计
> `/Laser_map`：当前 `tools/setup_mid360_dependencies.bash` 不应用
> `patches/fast_lio_local_map.patch`，实际生效的是手持与时间戳保护补丁。
> 当前地图输出、保存语义和调参入口以 [真实 MID-360 README](../README_REAL_MID360.md)
> 为准。下文“本次”“当前”均指 2026-09-23 局部地图实验当时。
> 该局部地图补丁与当前手持累计模式修改了同一段源码，不能在当前已打
> 手持补丁的 FAST-LIO 检出上直接 `git apply`；需要先另行合并、验证。

## 问题与本次范围

Xju FAST_LIO 440a8e3 的 publish_map 每秒把最新扫描变换后追加到
pcl_wait_pub；没有新扫描时也会再次追加最后一帧。累计内容不是估计器
维护的 ikd-tree 地图，会增加点数、通信和显示负担。

用户明确选择恢复官方地图语义。因此未部署额外的 bounded_map 节点，
也没有更换估计器、外参、IMU 配置或体素参数。

## 文件与修改

- `../FAST_LIO/src/laserMapping.cpp`（路径相对仓库根目录）：
  删除 pcl_wait_pub；publish_map 在 1 Hz 定时回调中 flatten ikd-tree，
  发布 featsFromMap，保留 ROS 2 odom 帧；树未初始化时不发布。
  移除原来 if(0) 的重复导出代码；/Laser_map 发布队列 20 → 1。
  /map_save 改为提取并保存当前局部地图，检查空地图与写入失败。
- `patches/fast_lio_local_map.patch`：上述依赖源码的可复现补丁。
- `tools/setup_mid360_dependencies.bash`：锁定提交后幂等应用新补丁。
- `go2_fastlio_localization/config/real_mapping.rviz`：两路点云改为
  Points、2 像素，队列均 1，Decay Time 保持 0；彩色显示改名 Local ikd-tree Map。
- `go2_fastlio_localization/go2_fastlio_localization/save_live_map.py`：
  启动立即提示保存的是局部快照，修正超时错误描述；保存命令不变。
- `README_REAL_MID360.md`：更新话题、保存语义、参数边界。

官方参考：<https://github.com/hku-mars/FAST_LIO/blob/main/src/laserMapping.cpp>。
官方默认关闭 flatten 和 publish_map，这里显式启用，并按 ROS 2 fork 的帧名发布。
提取与算法回调处于默认互斥 callback group，当前程序采用单线程 spin；
没有新增跨回调访问树的线程。

## 验证

- fast_lio、go2_fastlio_localization Release 编译成功；只有 Boost 弃用提示。
- shell 语法、Python 编译、git diff --check、补丁反向 check 通过。
- 现有 `bags/mid360_stationary`（约 21.6 秒）在独立 ROS_DOMAIN_ID=87
  离线回放，不启动驱动/RViz。输入结束后继续订阅 6 秒。
- 复测接收到 5 条地图，最后 4 条点数均 738，单条数据 35424 字节，
  frame_id=odom；不再在停止输入后重复累加。测试节点已关闭。
- 首次测试未收到地图；在启动 Python 前统一 ROS_DOMAIN_ID 与
  ROS_LOCALHOST_ONLY 后复测通过。不要在不同终端混用不同发现配置。
- 未验证本次真实房间移动扫描的墙厚或厘米级误差。

## 重要边界

filter_size_surf/map 仍为 0.5 m，cube_side_length 仍为 1000 m。
发布的是算法内部降采样的局部地图，不是保证每体素严格一个点的全局网格。
不存在固定内存上限，地图在探索新区域时仍会增长，受内部窗口管理。
如果缩小窗口，必须注意 cube_side_length > 3 * det_range（当前 det_range=40）。
删除历史点或显示成小点不等于消除姿态漂移。当前没有新增回环、动态物体
剔除、平面拟合或坐标修正。当前白色扫描与彩色地图若持续错位，应单独排查。
Path 的历史轨迹仍由上游维护，这次仅解决地图累积出口。

save_live_map 保存的是局部快照，不保证覆盖所有走过区域。
pcd_save_en 默认仍 false，开启后上游还有单独的扫描落盘缓存增长路径。
代码备份：`/home/ice/go2_ws/dependency_backups/local_map_20260923/`。
未提交或推送 GitHub。

## 后续：0.3 m 与 RViz 卡顿反馈

用户指定 filter_size_surf/map 均改为 0.3 m，已构建配置包。
相同录包再次回放：接收 12 条局部地图，最终 1808 点、86784 字节，
停止输入后的四次点数均为 1808。此结果不是实时房间扫描精度验证。
用户反馈窗口一打开就卡，检查时程序已退出且雷达网线已拔掉。
上次日志显示算法已初始化地图，但无地图点数或帧耗时记录，无法直接定位卡顿。
空场景硬件渲染测试内存约 238 MB，软件约 201 MB，后者 CPU 约 77%，
高于硬件；没有据此切换软件渲染，也没有确认图形驱动就是根因。
作为显示减负，默认关闭 Registered Cloud、TF、Path 显示，保留局部地图
和当前 Odometry，帧率从 30 降为 15。发布与保存不变。
需重新连接雷达后验证真实输入时的 RViz 响应；不能宣称已解决窗口卡顿。

## 2026-09-23 真实雷达密度对比

雷达 192.168.1.111 经 WSL 有线网卡 192.168.1.50 连通。
每组单独启动驱动和 FAST-LIO，无 RViz，约 18 秒读数。场景与雷达
可见区域未做严格控制，点数只能作调参依据，不能当作精度测量。

| point_filter_num | filter_size_surf/map | 原始点中位数/帧 | 配准点中位数/帧 | 最后局部地图点数 | 建图 RSS |
| --- | --- | ---: | ---: | ---: | ---: |
| 3 | 0.5 m | 18432 | 3396 | 1533 | 未采样 |
| 2 | 0.3 m | 19104 | 5327 | 2991 | 212 MB |
| 2 | 0.2 m | 18528 | 5193 | 4752 | 174 MB |
| 2 | 0.1 m | 18528 | 5405 | 17202 | 185 MB |

最终采用 point_filter_num=2、两项体素=0.1 m。0.1 m 组收到
约 10 Hz 原始/配准数据，局部地图约 1 Hz。进程负载是短时采样，
地图与内存仍可能随覆盖范围增长。RViz 用该组实流短测约 20 秒，
进程约 251 MB，未见崩溃；仍需用户实际拖动窗口确认响应。

参考图 `doc/results/HKU_MB_002.png` 是该 fork README 的大型场景
rosbag 示例，图像本身没有说明使用局部 `/Laser_map` 还是累计 PCD。
当前室内局部地图的规模和覆盖面积不能直接与该示例图相比。
