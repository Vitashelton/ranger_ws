# Repository Audit — Ranger Mini 2.0 + ROS2/Nav2

**Date**: 2026-05-15
**Auditor**: Automated review of `/home/zbx/jetson_wsss`

---

## 1. Repository Structure

```
/home/zbx/jetson_wsss/
├── src/
│   ├── ugv_sdk/                 # AgileX/Weston Robot CAN SDK (C++)
│   ├── ranger_ros2/             # Official ROS2 wrapper for Ranger
│   │   ├── ranger_msgs/         # Custom ROS2 messages
│   │   ├── ranger_base/         # Base driver node + messenger
│   │   └── ranger_bringup/      # Launch files for bringup
│   └── ranger_nav/              # Custom nav package (your work)
│       ├── launch/              # 6 launch files (base, sensors, slam, nav, 3d_slam, full)
│       ├── config/              # 5 YAML configs (nav2, slam×2, fastlio, pcl_to_scan)
│       ├── rviz/                # 2 RViz configs (nav, 3d_slam)
│       ├── scripts/             # can_bringup.sh, setup_ranger_nav.sh, pcd_to_2d_map.py
│       └── doc/                 # GIT_GUIDE.md, RANGER_NAV_TUTORIAL.md
├── build/
├── install/
└── log/
```

---

## 2. Current Capabilities (What Works)

### 2.1 Chassis CAN Communication
- **SDK**: `ugv_sdk` (AgileX protocol v2) with `RangerRobot` class supporting V1/V2/V3 variants.
- **CAN setup**: `scripts/can_bringup.sh` at `/home/zbx/jetson_wsss/src/ranger_nav/scripts/can_bringup.sh:1` uses `gs_usb` kernel module, default `can1` at 500kbps.
- **Connection**: `ranger_messenger.cpp:69-78` — `robot_->Connect(port_name_)` then `EnableCommandedMode()`.

### 2.2 /cmd_vel Control
- **Subscriber**: `ranger_messenger.cpp:194-196` subscribes to `/cmd_vel` (geometry_msgs/Twist).
- **Mode mapping** in `TwistCmdCallback` (lines 410-516):
  - `vx + wz` (no vy) → Ackermann mode (`SetMotionMode(kDualAckerman)`)
  - `vx + vy` (with or without wz) → Parallel/oblique mode (`SetMotionMode(kParallel)`)
  - Pure `wz` (no vx) → Spinning mode (`SetMotionMode(kSpinning)`)
  - Full stop when `|v| < 1e-4` and `|w| < 1e-4`
- **Sign convention**: When `vy` present and `wz` also present, `wz` is dropped with a throttled warning (line 443-448).
- **Steering angle calculation**: `CalculateSteeringAngle` at line 524 computes from `linear/angular` with `atan(wheelbase/2 / radius)`.

### 2.3 /odom Publishing
- **Publisher**: `ranger_messenger.cpp:189` publishes `nav_msgs/Odometry` on configurable topic (default `/odom`).
- **Kinematics models**: `kinematics_model.hpp` contains three models:
  - `DualAckermanModel` (line 31): `dx = v·cos(φ)·cos(θ)`, `dy = v·cos(φ)·sin(θ)`, `dθ = 2·v·sin(φ)/L`
  - `ParallelModel` (line 55): `dx = v·cos(θ+φ)`, `dy = v·sin(θ+φ)`, `dθ = 0`
  - `SpinningModel` (line 78): `dx = 0`, `dy = 0`, `dθ = w`
- **Odom message**: `UpdateOdometry` at line 316 — publishes `linear.x` and `linear.y` in parallel/side-slip modes (line 384-385), but only `linear.x` in Ackermann mode (line 373-375).
- **TF**: `publish_odom_tf_` parameter controls `odom → base_link` TF broadcast (line 395-407).

### 2.4 System State Publishing
All working through `ranger_messenger.cpp:181-191`:
| Topic | Type | Content |
|-------|------|---------|
| `/system_state` | `ranger_msgs/SystemState` | vehicle_state, control_mode, error_code, battery_voltage, motion_mode |
| `/motion_state` | `ranger_msgs/MotionState` | motion_mode enum |
| `/actuator_state` | `ranger_msgs/ActuatorStateArray` | 8 actuator states (driver + motor) |
| `/battery_state` | `sensor_msgs/BatteryState` | voltage, temperature, current, SOC (normalized 0~1) |

### 2.5 SLAM
- **2D**: `slam_toolbox` (online async mapping + localization mode), configured in `slam_toolbox_mapping.yaml` and `slam_toolbox_localization.yaml`.
- **3D**: FAST-LIO2 (LiDAR-IMU odometry), configured in `fastlio_mid360.yaml`. Uses MID360 built-in IMU.
- **PCD→2D map**: `scripts/pcd_to_2d_map.py` converts 3D PCD to Nav2-compatible PGM+YAML.

### 2.6 Nav2 Stack
- **Launch**: `ranger_nav.launch.py` starts map_server, controller_server, planner_server, behavior_server, bt_navigator, waypoint_follower, lifecycle_manager.
- **Planner**: NavfnPlanner (`nav2_navfn_planner/NavfnPlanner`).
- **Controller**: DWB (`dwb_core::DWBLocalPlanner`).
- **Behaviors**: spin, backup, wait.
- **Localization**: slam_toolbox in localization mode (not AMCL).

### 2.7 Costmap
- **Global costmap** (`nav2_params.yaml:176-214`): static_layer + obstacle_layer (LaserScan) + inflation_layer. `rolling_window: false`.
- **Local costmap** (`nav2_params.yaml:216-258`): voxel_layer (LaserScan) + inflation_layer. `rolling_window: true`, 6m × 6m.
- **Inflation**: radius=0.55m, cost_scaling_factor=3.0.

### 2.8 Sensor Pipeline
- **MID360S**: livox_ros_driver2 → `/livox/lidar` (PointCloud2) → pointcloud_to_laserscan → `/scan` (LaserScan).
- **Static TF**: `base_link → livox_frame` at x=0.30, z=0.70 in `ranger_sensors.launch.py:22-35`.
- **No D435i integration present.**

---

## 3. Problems Identified (Referenced to Code)

### 3.1 Nav2 Configured as Differential Drive (CRITICAL)

**File**: `src/ranger_nav/config/nav2_params.yaml`

- **Line 32**: `robot_model_type: nav2_amcl::DifferentialMotionModel` — AMCL uses differential motion model, meaning the particle filter cannot propagate lateral (vy) motion. This causes localization drift when the robot uses oblique/side-slip modes.
- **Line 92**: `min_vel_y: 0.0`
- **Line 94**: `max_vel_y: 0.0`
- **Line 106**: `vy_samples: 0`
- **Lines 100-104**: `acc_lim_y: 0.0`, `dec_lim_y: 0.0`

**Impact**: DWB generates zero lateral velocity candidates. Nav2 treats Ranger Mini 2.0 as a differential-drive robot with no omnidirectional capability.

### 3.2 AMCL Uses slam_toolbox Localization Instead

**File**: `src/ranger_nav/launch/ranger_nav.launch.py:53-59`

The nav launch uses `slam_toolbox` in localization mode, not AMCL. This means the AMCL `DifferentialMotionModel` setting at nav2_params.yaml:32 is not actually used in the current configuration. However, if AMCL is ever enabled (e.g., for comparison experiments), it must be switched to `OmniMotionModel`.

### 3.3 Odometry Does Not Report linear.y in All Modes

**File**: `src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:373-378`

In Ackermann mode, the odometry only reports `linear.x` (line 374) and computes `angular.z` from Ackermann kinematics (line 376-378). `linear.y` is always `0.0` in this mode (line 375).

**Impact**: When the robot executes an oblique command (vx+vy from Nav2), the odometry correctly reports both `linear.x` and `linear.y` (lines 384-385). But when running standard Ackermann, only `linear.x` is non-zero, which is correct for that mode. The concern is that Nav2 always sees `linear.y ≈ 0` in odom during normal driving, so it never learns the robot can move laterally.

### 3.4 cmd_vel → Chassis Mode Mapping Limitation

**File**: `src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:439-472`

When `vy` is present, the code enters Parallel mode (line 440-441). If `wz` is also present, `wz` is dropped (warning at line 443-448). This is physically correct — the CAN protocol cannot execute simultaneous translation and rotation in oblique mode.

However, Nav2 DWB as currently configured never outputs `vy`, so this code path is effectively unused during navigation. The mode switching logic exists and works, but DWB never triggers it.

### 3.5 No Footprint Defined

**File**: `src/ranger_nav/config/nav2_params.yaml:260-261`

Comment says: "This is set in launch via robot_radius or footprint parameter" — but examining all launch files (`ranger_nav.launch.py`, `ranger_full.launch.py`), no footprint or robot_radius is actually passed. The default Nav2 footprint is likely a point (0 radius), which is unsafe for a 0.55m × 0.45m robot.

### 3.6 No Dynamic Obstacle Detection / Tracking / Prediction

The entire perception pipeline is:
```
/livox/lidar → pointcloud_to_laserscan → /scan → costmap obstacle_layer
```

There is no:
- Dynamic obstacle detection (moving vs static)
- Multi-frame obstacle tracking
- Velocity estimation
- Trajectory prediction
- TTC computation
- Risk-aware trajectory evaluation

### 3.7 No Replanning Trigger Beyond Nav2 Default Behaviors

Nav2 behavior server has `spin`, `backup`, `wait` recovery behaviors. But there is no:
- Detection of local planner repeated failures
- Robot stuck/oscillation detection
- Global path obstruction monitoring
- Automatic `clear costmap` or `ComputePathToPose` trigger
- Emergency stop based on dynamic risk

### 3.8 No Sensor Fusion (MID360S Only)

Only MID360S is integrated. No D435i depth camera in the pipeline. The tutorial acknowledges this at `RANGER_NAV_TUTORIAL.md:279`: "Phase 3 接入 D435i 深度相机解决" but no implementation exists.

### 3.9 Hardcoded Paths

| File | Hardcoded Path | Line |
|------|---------------|------|
| `ranger_sensors.launch.py` | `/home/robot/livox_ws/...` | 39-41 |
| `ranger_nav.launch.py` | `/home/robot/maps/ranger_map.yaml` | 24, 152 |
| `slam_toolbox_mapping.yaml` | `/home/robot/maps/ranger_map` | 22 |
| `slam_toolbox_localization.yaml` | `/home/robot/maps/ranger_map` | 19 |
| `ranger_full.launch.py` | `/home/robot/maps/ranger_map.yaml` | 35 |

### 3.10 No Battery Safety Auto-Stop

**File**: `src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:306-310`

Low battery warning is logged but no automatic stop or mode restriction. Below 15% SOC, a warning is throttled at 5s — but motor commands continue.

### 3.11 No Mode Switch Cooling Time

**File**: `src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:410-516`

Every `TwistCmdCallback` call can switch the motion mode. Rapid oscillation between `vx=0` and `vy≠0` (e.g., from DWB) could cause frequent mode switching, which wears actuators. No minimum time between mode switches is enforced.

### 3.12 steering_angle Sign in ranger_messenger.cpp vs upstream

**File**: `src/ranger_ros2/ranger_base/src/ranger_messenger.cpp:539`

```cpp
const int sign = (msg.angular.z * msg.linear.x) >= 0.0 ? 1 : -1;
```

This keeps the upstream sign convention (line 538 comment). For right-hand ROS convention (positive z = left turn), with positive vx, positive wz should steer left = positive angle. The current formula gives `sign = +1` when `wz * vx >= 0`, meaning `positive wz * positive vx → positive steer`. This is correct for a standard ROS differential drive, but should be verified on the physical Ranger Mini 2.0.

### 3.13 slam_toolbox vs AMCL for Final Paper

Using slam_toolbox for localization is fine for demos, but for a research paper comparing against standard Nav2 baselines, AMCL is the standard localization method and should be included as a baseline option. The current nav2_params.yaml has AMCL parameters but they're unused.

---

## 4. Summary of Gaps vs Paper Requirements

| Requirement | Status | Action |
|-------------|--------|--------|
| Mode-aware chassis interface | Partially done (TwistCmdCallback exists) | Add cooling time, formalize priority, add lateral capability to Nav2 |
| Nav2 omni-mode DWB | Not enabled (vy=0) | Enable vy samples, test oblique avoidance |
| Dynamic obstacle detection | Not present | New package: ranger_dynamic_obstacle |
| Risk-aware trajectory evaluation | Not present | Add risk critics to DWB or implement custom controller plugin |
| Replanning trigger + recovery | Not present | New package: ranger_replan_manager |
| Multi-sensor fusion (MID360S + D435i) | Not present | New module: sensor fusion, TF, D435i pipeline |
| Experiment logging + metrics | Not present | New package: ranger_nav_metrics |
| Paper-ready evaluation pipeline | Not present | rosbag recording, offline analysis scripts |
