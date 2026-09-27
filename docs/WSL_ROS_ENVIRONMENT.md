# WSL ROS environment check — 2026-09-24

Use the default ROS domain (0), default non-localhost-only discovery, and default Humble RMW (rmw_fastrtps_cpp). No per-terminal export is needed. Existing terminals previously using overrides should run `unset ROS_DOMAIN_ID ROS_LOCALHOST_ONLY` once, then source the workspace.

Observed: no ROS daemon or other listener occupied TCP 11511 initially. Requests to unused 127.0.0.1:11511 and :11512 waited until the diagnostic timeout. Standard ros2 daemon status and node list therefore stalled during their XML-RPC probe. Directly starting the standard daemon restored response. The exact network/filter layer causing the missing connection refusal remains unconfirmed; WSL uses mirrored networking.

Workaround installed: ~/.config/systemd/user/ros2-daemon.service. Starts the standard Humble daemon in domain 0 with default discovery; Restart=always also restarts it after its idle timeout. This avoids the empty-port probe during normal use, but does not repair the underlying network behavior. Manage with `systemctl --user restart ros2-daemon`; stop and disable with `systemctl --user disable --now ros2-daemon`. A plain `ros2 daemon stop` is temporary because the service restarts it. When deliberately changing domain or middleware, disable this service first or update it accordingly.

VS Code WSL settings: remote.autoForwardPorts=false; remote.restoreForwardedPorts=false. Previous settings saved beside settings.json as settings.before-ros-fix.json. Existing forwarded entries may require stopping forwarding in the Ports panel. The reported 17 entries have not been individually identified. Do not equate entries with running ROS nodes.

README_REAL_MID360.md startup instructions now clear old domain/localhost overrides. Historical documents remain unchanged. No mapping parameters or system ROS source were modified. No reboot performed.

Validation: default-environment daemon status returns; temporary demo talker publishes, topic info reports one publisher, no-daemon discovery lists environment_check_talker. Discovery takes time after node startup. Test talker is bounded by timeout and exits automatically.

## 2026-09-27：WSL 时钟周期性回拨

这是 **WSL 主机环境设置**，不是 ROS 参数、FAST-LIO 参数或仓库代码改动。
它只适用于这台 Windows 主机内的 Ubuntu-22.04 WSL 实例；把本仓库推到
GitHub 不会把 systemd 服务状态一起部署到别人的 WSL，更不能照搬到原生
Ubuntu 22.04 的机器人电脑。原生 Ubuntu 需要自行保持可靠的时间同步。

### 证据与范围

- 2026-09-26 的诊断包
  `~/go2_bags/tilt_diagnostic_20260926_131440/`（数据不在 Git 仓库中）
  按消息写入顺序检查，LiDAR、IMU、`/Odometry` 和配准点云的消息头时间戳
  单调递增；倒退的是 bag 接收时间／WSL 系统时间。86 秒记录中观察到
  3 次约 0.8～1.0 秒的接收时间回拨，`systemd-resolved` 同期反复记录
  `Clock change detected`。这与前一天旧版驱动录包中的 IMU 消息戳倒退
  不是同一次事件，不能混为一个结论。
- GICP 使用 `now() - cloud.header.stamp` 检查新鲜度。系统时间回拨可让
  该值变为负数，触发 `STALE_INPUT`，进而中断该次定位更新；这不等于
  已证明所有点云倾斜都由时钟造成。
- WSL 的 `systemd-timesyncd` 当时处于 `enabled/active`，向
  `ntp.ubuntu.com` 校时；WSL 也由 Hyper-V 从 Windows 同步时钟。
  Canonical 指出 Ubuntu 24.04 及更早的 WSL 版本可因此出现双重校时冲突：
  <https://ubuntu.com/wsl/docs/stable/explanation/time-sync/>。

### 本机所做的修改

在 **WSL Ubuntu-22.04**（不要在原生双系统 Ubuntu）执行：

```bash
sudo systemctl disable --now systemd-timesyncd.service
```

操作后 `systemctl is-enabled systemd-timesyncd.service` 为 `disabled`，
`systemctl is-active systemd-timesyncd.service` 为 `inactive`。
`disable` 阻止这份 WSL 下次自动启动该服务，`--now` 立即停用；
未修改 `/etc/systemd/timesyncd.conf`、Windows 注册表或 ROS 源码。

### 验证、边界与恢复

停用后，WSL 与 Windows 实测相差约 0～15 ms；在 180 秒监测中未发现
超过 0.1 秒的时钟跳变（此前约每 32 秒回拨一次）。这是当前环境的短时
验证，不保证休眠、Windows 校时或 WSL 更新后永不再跳。

Windows 本身在本次复测时比 `time.windows.com` 和 `time.nist.gov`
快约 1.66～1.68 秒，因此 WSL 跟随 Windows 后也可能有相同的绝对时间
误差。应在启动雷达/ROS 之前校准 Windows；不要在建图或定位运行中手动
同步，否则宿主机的一次跳时仍可能传给 WSL。多机协同时还需另行核对
所有机器的时钟。此修改不等于 GICP 运动定位或建图精度验收通过。

复查命令：

```bash
systemctl is-enabled systemd-timesyncd.service
systemctl is-active systemd-timesyncd.service
```

若要恢复 WSL 自己使用 NTP：

```bash
sudo systemctl enable --now systemd-timesyncd.service
```
