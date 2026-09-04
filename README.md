# Go2 MID-360: Gazebo 3D Mapping and Navigation

ROS 2 Humble project for Unitree Go2/CHAMP simulation with a Livox MID-360-style Gazebo Classic ray sensor, FAST-LIO 3D mapping, PCD-based ICP relocalization, and Nav2 navigation.

## Features

- Go2 and MID-360 simulation in Gazebo Classic 11;
- `/livox/lidar` PointCloud2, `/livox/lidar_custom` Livox CustomMsg, and a 2D `/scan` projection;
- FAST-LIO 3D mapping and PCD export;
- ICP relocalization in a saved PCD map;
- Nav2 planning and obstacle avoidance after a manually supplied approximate initial pose.

The navigation TF chain is sensor based:

```text
map -> odom -> base_footprint -> base_link -> mid360_link
```

`/odom` is generated from FAST-LIO `/Odometry`. The main FAST-LIO navigation launch does not use Gazebo `/odom/ground_truth`.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- A ROS workspace, for example `~/go2_ws`

## Installation

Clone this repository into `<workspace>/src`, then run:

```bash
cd ~/go2_ws/src/unitree-go2-ros2
chmod +x tools/setup_mid360_dependencies.bash
./tools/setup_mid360_dependencies.bash
```

The setup script installs ROS packages, clones pinned versions of Livox SDK2, `livox_ros_driver2`, `livox_laser_simulation_ros2`, and `FAST_LIO_ROS2`, applies the project patches, and builds the workspace.

## Workflows

### Build a 3D map

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_3d_mapping.bash gui:=true show_rviz:=true
```

Drive with `teleop_twist_keyboard`; when mapping is complete, stop the launch with `Ctrl+C`. The configured output is `~/go2_ws/maps/mid360_3d.pcd`.

### Create a Nav2 map from a PCD

```bash
python3 ~/go2_ws/src/unitree-go2-ros2/robots/configs/go2_config/scripts/pcd_to_nav2_map.py \
  ~/go2_ws/maps/mid360_3d.pcd ~/go2_ws/maps/mid360_3d_nav \
  --resolution 0.05 --z-min 0.35 --z-max 1.50 --padding 0.30 \
  --min-points 2 --dilation-cells 1
```

This creates the matching `mid360_3d_nav.pgm` and `mid360_3d_nav.yaml` files.

### Relocalize and navigate

```bash
cd ~/go2_ws
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash gui:=true show_rviz:=true
```

Wait for the stack to load. In RViz, use **2D Pose Estimate** to give an approximate position and heading, then use **Nav2 Goal**.

To use another scene, give it a matching PCD and 2D projection:

```bash
./src/unitree-go2-ros2/tools/run_fast_lio_nav2_demo.bash \
  gui:=true show_rviz:=true \
  pcd_map:=~/go2_ws/maps/example.pcd \
  nav_map:=~/go2_ws/maps/example_nav.yaml
```

## Limitations

- This is a Gazebo Classic ray-based MID-360 approximation, not a hardware-accurate Livox model or hardware driver.
- ICP needs a reasonable manual initial pose and can fail in highly symmetric scenes or after large map changes.
- Nav2 uses a 2D height slice projected from the 3D PCD for planning; FAST-LIO and ICP remain the 3D localization source.

## Reproducibility notes

The `patches/` directory contains the required small patches for the pinned Livox Gazebo simulator and FAST-LIO sources. They add the Livox CustomMsg output required by FAST-LIO and make Ctrl+C save the accumulated PCD map.

This project builds on Unitree Go2 descriptions, CHAMP, Livox SDK2, `livox_ros_driver2`, LCAS `livox_laser_simulation_ros2`, and `FAST_LIO_ROS2`. Their respective licenses apply to their source code.
