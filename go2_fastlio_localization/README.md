# go2_fastlio_localization

真实雷达当前操作、目录结构、参数和接口统一见
[真实 MID-360 使用说明](../README_REAL_MID360.md)。

本包的 real_lidar 负责驱动参数，real_fastlio 调用 Xju 新版 FAST-LIO 上游 launch，
real_mapping 是两者的可选组合。真实配置仅编辑 config/mid360_real.json、
config/fast_lio_mid360_real.yaml、config/real_mapping.rviz。

本包同时保留旧仿真、ICP 和导航桥接代码。新版坐标系改变，旧导航链尚未适配验收。
