cmake_minimum_required(VERSION 3.5)
project(go2_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config maps worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/fast_mid360_scan.py
    scripts/fast_mid360_3d.py
    scripts/check_mid360_3d.py
    scripts/drive_3d_mapping_route.py
    scripts/drive_3d_mapping_east.py
    scripts/ground_truth_odom.py
    scripts/fast_lio_odom_bridge.py
    scripts/pcd_icp_localizer.py
    scripts/pcd_to_nav2_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
