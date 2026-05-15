# RangerNav-Fusion

## Mode-Aware and Multi-Sensor Risk-Aware Navigation for Four-Wheel-Steering UGVs

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-1.1-green)](https://navigation.ros.org/)
[![Platform](https://img.shields.io/badge/Platform-Ranger%20Mini%202.0-orange)](https://www.westonrobot.com/ranger-mini-2)

RangerNav-Fusion extends ROS2 Navigation2 for the **Ranger Mini 2.0 four-wheel-steering UGV**, adding mode-aware chassis control, MID360S+D435i multi-sensor dynamic obstacle perception, confidence-aware risk evaluation, and closed-loop replanning.

---

## 1. Project Overview

Standard Nav2 treats all robots as velocity-space controllable — just set `vx`, `vy`, `wz`. Ranger Mini 2.0 doesn't work that way. It has **discrete motion modes** (Ackermann, oblique, spin) with different kinematic constraints. You can't do arbitrary `vx+vy+wz` simultaneously through the CAN bus.

RangerNav-Fusion bridges this gap with three complementary layers:

```
Nav2 (planner + controller)
        │
   ┌────┴────┐
   │  RISK   │ ← TTC, confidence, prediction-aware trajectory scoring
   ├─────────┤
   │ FUSION  │ ← MID360S (360° LiDAR) + D435i (RGB-D near-field)
   ├─────────┤
   │  MODE   │ ← cmd_vel → {Ackermann, Oblique, Spin} dispatch
   └────┬────┘
        │
  Ranger Mini 2.0 (CAN bus)
```

---

## 2. Hardware Platform

| Component | Model | Purpose |
|-----------|-------|---------|
| **UGV** | Ranger Mini 2.0 | 4WS chassis, 0.55×0.45m, max 1.5 m/s |
| **LiDAR** | Livox MID360S | 360° SLAM + mid/long-range obstacle detection |
| **Camera** | Intel RealSense D435i | Near-field blind zone + low obstacle detection |
| **Compute** | NVIDIA Jetson Orin | Onboard processing |
| **Interface** | CAN bus (gs_usb) | Chassis communication |

## 3. Software Dependencies

- ROS2 Humble
- Nav2 (navigation2)
- slam_toolbox (2D SLAM + localization)
- FAST-LIO2 (3D LiDAR-IMU SLAM, optional)
- livox_ros_driver2 (MID360S driver)
- realsense2-camera (D435i driver)
- pointcloud_to_laserscan
- ugv_sdk + ranger_ros2 (chassis SDK)

## 4. Build

```bash
# Prerequisites
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox ros-humble-pointcloud-to-laserscan
sudo apt install ros-humble-realsense2-camera ros-humble-depth-image-proc
sudo apt install libasio-dev libboost-all-dev

# Create workspace
mkdir -p ~/ranger_nav_ws/src
cd ~/ranger_nav_ws/src

# Clone dependencies
git clone https://github.com/agilexrobotics/ugv_sdk.git
git clone https://github.com/agilexrobotics/ranger_ros2.git

# Setup MID360S driver
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd ~/livox_ws
colcon build --symlink-install

# Build main workspace
cd ~/ranger_nav_ws
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
colcon build --symlink-install
```

## 5. CAN Configuration

```bash
# One-time (after power cycle):
sudo modprobe gs_usb
sudo ip link set can1 down 2>/dev/null
sudo ip link set can1 type can bitrate 500000 restart-ms 100
sudo ip link set can1 up

# Verify
candump can1
```

## 6. Quick Start

### Mapping (2D SLAM)
```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/ranger_nav_ws/install/setup.bash

# One-click SLAM
ros2 launch ranger_nav ranger_full.launch.py mode:=mapping

# Save map
ros2 run nav2_map_server map_saver_cli -f /home/robot/maps/ranger_map
```

### Mapping (3D SLAM, FAST-LIO2)
```bash
ros2 launch ranger_nav ranger_full.launch.py mode:=mapping3d

# Convert to 2D map for Nav2
python3 src/ranger_nav/scripts/pcd_to_2d_map.py saved.pcd ranger_3d_map \
    --resolution 0.05 --z_min 0.2 --z_max 2.0 --dilate 2
```

### Navigation
```bash
ros2 launch ranger_nav ranger_full.launch.py mode:=nav map:=/home/robot/maps/ranger_map.yaml
```

### Keyboard Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 7. Step-by-Step Launch (Debugging)

```bash
# Terminal 1: Chassis only
ros2 launch ranger_nav ranger_base.launch.py

# Terminal 2: Sensors
ros2 launch ranger_nav ranger_sensors.launch.py

# Terminal 3: SLAM (mapping mode)
ros2 launch ranger_nav ranger_slam.launch.py

# Or: Navigation
ros2 launch ranger_nav ranger_nav.launch.py map:=/home/robot/maps/ranger_map.yaml
```

## 8. Dynamic Obstacle Experiments

### Run with sensor fusion + risk evaluation
```bash
# (Planned: full pipeline)
ros2 launch ranger_nav ranger_full.launch.py mode:=nav \
    controller:=omni_dwb \
    risk_evaluation:=true \
    sensor_fusion:=true \
    map:=/home/robot/maps/ranger_map.yaml
```

### Record experiment data
```bash
ros2 bag record \
  /odom /cmd_vel /plan /local_plan /scan /tf /tf_static \
  /goal_pose /system_state /motion_state \
  /obstacles_mid360 /obstacles_d435i /fused_obstacles \
  /tracked_obstacles /predicted_obstacles /risk_markers \
  /replan_event /navigation_metrics \
  -o experiment_$(date +%Y%m%d_%H%M%S)
```

## 9. Project Structure

```
ranger_nav/
├── launch/
│   ├── ranger_base.launch.py          # Chassis driver only
│   ├── ranger_sensors.launch.py       # MID360S + static TF + pcl_to_scan
│   ├── ranger_slam.launch.py          # 2D SLAM (slam_toolbox)
│   ├── ranger_3d_slam.launch.py       # 3D SLAM (FAST-LIO2)
│   ├── ranger_nav.launch.py           # Nav2 stack
│   └── ranger_full.launch.py          # One-click: chassis+sensors+SLAM/nav
├── config/
│   ├── nav2_params.yaml               # Nav2 config (differential baseline)
│   ├── slam_toolbox_mapping.yaml      # 2D SLAM params
│   ├── slam_toolbox_localization.yaml # Localization mode params
│   ├── fastlio_mid360.yaml            # FAST-LIO2 config for MID360
│   └── pointcloud_to_laserscan.yaml   # 3D→2D scan conversion
├── rviz/
│   ├── ranger_nav.rviz                # 2D navigation RViz config
│   └── ranger_3d_slam.rviz            # 3D SLAM RViz config
├── scripts/
│   ├── can_bringup.sh                 # CAN bus setup
│   ├── setup_ranger_nav.sh            # Workspace sourcing
│   └── pcd_to_2d_map.py               # 3D PCD→2D grid map converter
├── doc/
│   ├── RANGER_NAV_TUTORIAL.md         # Tutorial (Chinese)
│   └── GIT_GUIDE.md                   # Git guide (Chinese)
├── ranger_sensor_fusion/              # [Planned] Multi-sensor fusion
├── ranger_dynamic_obstacle/           # [Planned] Tracking + prediction
├── ranger_replan_manager/             # [Planned] Replan trigger
└── ranger_nav_metrics/                # [Planned] Metrics + analysis
```

## 10. Research Direction (Paper)

**Title**: RangerNav-Fusion: Mode-Aware and Multi-Sensor Risk-Aware Navigation for Four-Wheel-Steering UGVs in Dynamic Environments

**中文**: 面向动态环境的四轮四转移动机器人运动模式感知与风险感知导航方法

**Contributions**:
1. Mode-aware chassis interface mapping Nav2 `cmd_vel` to 4WS motion modes
2. Lightweight MID360S-D435i sensor fusion for dynamic obstacle perception
3. Confidence-aware TTC-based risk evaluation for trajectory scoring
4. Global-local replanning pipeline with recovery escalation

**Documentation**:
- `docs/repo_audit.md` — Current repository analysis
- `docs/rangernav_risk_project_plan.md` — Project architecture and milestones
- `docs/method_design.md` — Algorithm design (mode mapping, perception, risk, replanning)
- `docs/implementation_plan.md` — Incremental development plan (MVP-1 to MVP-9)
- `docs/nav2_config_recommendations.md` — Nav2 parameter analysis and recommendations
- `docs/experiment_plan.md` — 15 experiment designs with metrics
- `docs/sensor_setup.md` — MID360S + D435i hardware setup
- `docs/mid360_d435i_fusion.md` — Sensor fusion algorithm design
- `docs/tf_calibration_checklist.md` — TF calibration procedure
- `docs/sensor_ablation_experiments.md` — Sensor ablation study design
- `paper/rangernav_risk_paper_draft.md` — Paper draft (all results [TBD])

## 11. TODO

### MVP-1: Baseline Navigation ✓
- [x] CAN communication working
- [x] MID360S → /scan → Nav2 working
- [x] SLAM (2D + 3D) working
- [x] Launch files organized

### MVP-2: D435i Integration
- [ ] Create `d435i_sensor.launch.py`
- [ ] Create `config/d435i_filter.yaml`
- [ ] Add D435i scan to local costmap
- [ ] Test near-field obstacle detection

### MVP-3: Sensor Fusion Visualization
- [ ] Create `ranger_sensor_fusion` package
- [ ] `obstacle_cluster_node.py` (MID360S)
- [ ] `d435i_obstacle_node.py` (D435i)
- [ ] `sensor_fusion_node.py` (association + merging)
- [ ] RViz Marker visualization

### MVP-4: Tracking + Prediction + Risk
- [ ] Create `ranger_dynamic_obstacle` package
- [ ] `obstacle_tracker_node.py` (Kalman filter)
- [ ] `obstacle_predictor_node.py` (constant velocity)
- [ ] `risk_evaluator_node.py` (TTC + confidence)
- [ ] Risk Marker visualization

### MVP-5: Replan Manager
- [ ] Create `ranger_replan_manager` package
- [ ] `replan_manager_node.py` (failure detection + recovery)
- [ ] Integration test with Nav2

### MVP-6: Omni DWB
- [ ] Enable vy_samples in nav2_params.yaml
- [ ] Test lateral obstacle avoidance
- [ ] Tune mode_switch_cooldown

### MVP-7: MPPI Comparison
- [ ] Create `nav2_risk_mppi.yaml`
- [ ] A/B test DWB vs MPPI

### MVP-8: Metrics
- [ ] Create `ranger_nav_metrics` package
- [ ] `metrics_logger.py`, `bag_analyzer.py`, `plot_results.py`

### Experiments
- [ ] Run 15+ experiment scenarios
- [ ] Collect rosbags
- [ ] Analyze results
- [ ] Fill paper [TBD] fields

## 12. Safety Notes

1. **Always keep remote control accessible** — Ranger Mini 2.0 remote has a physical e-stop.
2. **Test at low speed first** — Use `max_vel_x: 0.5` for initial testing.
3. **Monitor `/system_state` error codes** — Non-zero error means CAN communication issues.
4. **Low battery** — Ranger Mini 2.0 powers steering motors when idle. Monitor SOC and charge regularly.
5. **MID360S blind zone** — 0.5m around the LiDAR is invisible. Keep clear during startup.
6. **CAN cable** — Secure the CAN-USB adapter during experiments. Loose connection causes control loss.
7. **D435i IR safety** — The IR projector is Class 1 (eye-safe) but avoid prolonged direct exposure.

---

## License

MIT License

## Acknowledgments

- Ranger Mini 2.0 platform: AgileX Robotics / Weston Robot
- ugv_sdk and ranger_ros2: [agilexrobotics](https://github.com/agilexrobotics)
- Nav2: [ros-navigation](https://github.com/ros-navigation)
- FAST-LIO2: [Ericsii/FAST_LIO_ROS2](https://github.com/Ericsii/FAST_LIO_ROS2)
