# Implementation Plan — RangerNav-Fusion

## Principle: Incremental Development

Each MVP produces a working, testable system. No big-bang rewrite.

---

## Phase 0: Pre-requisites (No Code Changes)

### 0.1 Verify Current Baseline
```bash
# Record baseline navigation with current config
ros2 launch ranger_nav ranger_full.launch.py mode:=nav
ros2 bag record -o baseline_nav /odom /cmd_vel /plan /local_plan /scan /tf /tf_static /goal_pose
# Run 3 navigation tasks, save bags for later comparison
```

### 0.2 Create Branches
```bash
git checkout -b feat/mvp1-baseline-nav
git checkout -b feat/mvp2-d435i-costmap
git checkout -b feat/mvp3-fusion-viz
git checkout -b feat/mvp4-risk-markers
git checkout -b feat/mvp5-replan-trigger
# etc.
```

---

## Phase 1: MVP-1 — MID360S → /scan → Nav2 Works

**Goal**: Verify current setup works reliably before adding complexity.

### Files to Check/Update:

| File | Action | Reason |
|------|--------|--------|
| `ranger_nav/config/nav2_params.yaml` | Add robot footprint | Currently missing (see audit §3.5) |
| `ranger_nav/config/nav2_params.yaml` | Add `max_vel_x: 1.0` safety limit | Hard cap for real-robot safety |
| `ranger_nav/config/pointcloud_to_laserscan.yaml` | Tune `min_height`, `max_height` | Verify scan quality in test environment |

### Minimum Changes to nav2_params.yaml:

```yaml
# In controller_server → FollowPath:
max_vel_x: 0.8   # reduced for safe testing
max_vel_theta: 0.8  # reduced

# Add to local_costmap and global_costmap:
robot_radius: 0.35   # conservative: covers 0.55×0.45 rectangle
# OR explicit footprint:
footprint: "[[-0.275, -0.225], [0.275, -0.225], [0.275, 0.225], [-0.275, 0.225]]"
```

### Verification:
- [ ] Robot navigates to goal without collision
- [ ] `/scan` shows valid range data in RViz
- [ ] Costmap shows obstacles correctly
- [ ] rosbag recorded for baseline metrics

---

## Phase 2: MVP-2 — D435i Near-Field Obstacles → Local Costmap

**Goal**: Add D435i depth camera for near-field obstacle avoidance.

### New Files:

#### `ranger_nav/launch/d435i_sensor.launch.py`
```python
# Launches:
# 1. realsense2_camera_node (RGB-D)
# 2. depth_image_proc → pointcloud
# 3. pointcloud_to_laserscan (for costmap)
# 4. static TF: base_link → camera_link
```

#### `ranger_nav/config/d435i_filter.yaml`
```yaml
# PointCloud filtering for D435i near-field
min_range: 0.2
max_range: 4.0
min_height: 0.0   # include low obstacles
max_height: 1.5
voxel_leaf_size: 0.03
```

#### Updates to `ranger_nav/config/nav2_params.yaml`

Add D435i scan source to local costmap:
```yaml
local_costmap:
  ros__parameters:
    plugins: ["voxel_layer", "inflation_layer"]
    voxel_layer:
      observation_sources: scan d435i_scan
      d435i_scan:
        topic: /d435i/scan
        max_obstacle_height: 1.5
        clearing: true
        marking: true
        data_type: "LaserScan"
        raytrace_max_range: 4.0
        raytrace_min_range: 0.2
        obstacle_max_range: 4.0
        obstacle_min_range: 0.2
```

**Important**: D435i scan only goes to **local_costmap**, not global_costmap. This prevents D435i noise from corrupting the global map.

### Verification:
- [ ] D435i pointcloud visible in RViz
- [ ] Near-field obstacles (< 4m) appear in local costmap
- [ ] Robot stops for low obstacles that MID360S misses (e.g., water bottle on floor)
- [ ] No false positives from D435i noise

---

## Phase 3: MVP-3 — Fused Obstacles → RViz Visualization

**Goal**: Visualize fused obstacles from both sensors in RViz.

### New Package: `ranger_sensor_fusion` (Python)

#### Directory Structure:
```
src/ranger_nav/
└── ranger_sensor_fusion/
    ├── __init__.py
    ├── obstacle_cluster_node.py    # MID360S clustering
    ├── d435i_obstacle_node.py      # D435i near-field extraction
    ├── sensor_fusion_node.py       # Association + confidence
    └── launch/
        └── sensor_fusion.launch.py
```

#### `obstacle_cluster_node.py`
- **Input**: `/livox/lidar` (PointCloud2)
- **Output**: `/obstacles_mid360` (MarkerArray)
- **Params**: `cluster_tolerance`, `min_cluster_size`, `roi_min/max_x/y/z`, `voxel_leaf_size`
- **Frequency**: 10 Hz

#### `d435i_obstacle_node.py`
- **Input**: `/camera/depth/color/points` (PointCloud2)
- **Output**: `/obstacles_d435i` (MarkerArray), `/near_field_safety_zone` (Marker)
- **Params**: `max_range`, `min_height`, `cluster_tolerance`, `safety_zone_depth`
- **Frequency**: 15 Hz

#### `sensor_fusion_node.py`
- **Inputs**: `/obstacles_mid360`, `/obstacles_d435i`, `/odom`, `/tf`
- **Output**: `/fused_obstacles` (MarkerArray)
- **Fusion logic**:
  ```
  for each mid360_obs, d435i_obs pair:
      if distance(mid360_obs.center, d435i_obs.center) < association_threshold:
          → merge (weighted by sensor confidence)
      else:
          → keep as separate obstacle with source label
  ```
- **Confidence**:
  - MID360S alone: confidence = 0.8
  - D435i alone: confidence = 0.6
  - Both sensors agree: confidence = 0.95
  - MID360S > 10m: confidence = 0.7 (range falloff)
  - D435i > 3m: confidence = 0.4 (depth noise at range)

### Verification:
- [ ] Fused obstacles visible in RViz as colored markers
- [ ] MID360S obstacles (green), D435i obstacles (blue), fused (cyan)
- [ ] Obstacles at intersection of both FOVs show merged confidence
- [ ] `ros2 topic hz /fused_obstacles` ≥ 10 Hz

---

## Phase 4: MVP-4 — Tracking + Prediction + TTC Risk Markers

**Goal**: Track obstacles across frames, predict trajectories, compute TTC, visualize risk.

### New Package: `ranger_dynamic_obstacle` (Python)

#### Directory Structure:
```
src/ranger_nav/
└── ranger_dynamic_obstacle/
    ├── __init__.py
    ├── obstacle_tracker_node.py
    ├── obstacle_predictor_node.py
    ├── risk_evaluator_node.py
    └── launch/
        └── dynamic_obstacle.launch.py
```

#### `obstacle_tracker_node.py`
- **Input**: `/fused_obstacles` (MarkerArray)
- **Output**: `/tracked_obstacles` (MarkerArray, with IDs, velocities)
- **Method**: Kalman filter (4-state: px, py, vx, vy), Hungarian data association
- **Track lifecycle**: birth=3 detections, death=5 misses, confirmed=10 frames
- **Params**: `association_max_dist`, `birth_threshold`, `death_threshold`, `process_noise`, `meas_noise`

#### `obstacle_predictor_node.py`
- **Input**: `/tracked_obstacles`
- **Output**: `/predicted_obstacles` (MarkerArray, LINE_STRIP for predicted path)
- **Method**: Constant velocity extrapolation, 2.0s horizon at 0.2s steps
- **Output**: each track gets a LINE_STRIP marker showing future positions

#### `risk_evaluator_node.py`
- **Inputs**: `/tracked_obstacles`, `/predicted_obstacles`, `/odom`, `/plan`
- **Output**: `/risk_markers` (MarkerArray)
- **Computation**:
  ```
  for each obstacle:
      rel_vel = obstacle.vel - robot.vel  (closing speed)
      TTC = distance / max(-rel_vel_project, 0.01)
      risk_level:
          TTC > 3.0s → LOW (green marker)
          1.5s < TTC ≤ 3.0s → MEDIUM (yellow)
          0.5s < TTC ≤ 1.5s → HIGH (orange)
          TTC ≤ 0.5s → CRITICAL (red)
  ```

### Verification:
- [ ] Tracked obstacles maintain consistent IDs across frames
- [ ] Velocity estimates are reasonable (verify against manual measurement)
- [ ] Predicted trajectories shown as lines ahead of moving obstacles
- [ ] Risk markers change color with TTC
- [ ] Static obstacles show low/no risk, approaching obstacles show increasing risk

---

## Phase 5: MVP-5 — Replan Trigger

**Goal**: Detect navigation failures and trigger recovery.

### New Package: `ranger_replan_manager` (Python)

#### Directory Structure:
```
src/ranger_nav/
└── ranger_replan_manager/
    ├── __init__.py
    ├── replan_manager_node.py
    └── launch/
        └── replan_manager.launch.py
```

#### `replan_manager_node.py`
- **Subscriptions**: `/odom`, `/plan`, `/local_plan`, `/cmd_vel`, `/risk_markers`, `/system_state`
- **Timer**: 10 Hz evaluation loop
- **Recovery actions** (attempted in order):
  1. `clear_local_costmap` service call
  2. `clear_global_costmap` service call
  3. `compute_path_to_pose` service call
  4. Publish zero `/cmd_vel` (2s hold)
  5. Cancel navigation (publish to `/goal_pose` with cancel)

- **State machine**:
  ```
  NORMAL → (stuck/blocked) → SLOWDOWN
  SLOWDOWN → (recovered) → NORMAL
  SLOWDOWN → (still stuck) → CLEAR_LOCAL
  CLEAR_LOCAL → (recovered) → NORMAL
  CLEAR_LOCAL → (still stuck) → CLEAR_GLOBAL
  CLEAR_GLOBAL → (recovered) → NORMAL
  CLEAR_GLOBAL → (still stuck) → REPLAN
  REPLAN → (recovered) → NORMAL
  REPLAN → (still stuck) → WAIT
  WAIT → (recovered) → NORMAL
  WAIT → (still stuck, high risk) → SAFETY_STOP
  ```

### Verification:
- [ ] Robot stuck > 5s triggers slowdown → clear local → replan
- [ ] Dynamic obstacle blocking global path > 3s triggers replan
- [ ] TTC < 0.5s triggers immediate safety stop
- [ ] All events logged to `/replan_event`

---

## Phase 6: Enable Nav2 Omni Capability

### Changes to `nav2_params.yaml`:

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      min_vel_y: -0.3
      max_vel_y: 0.3
      vy_samples: 11          # 11 samples from -0.3 to 0.3
      acc_lim_y: 0.5
      dec_lim_y: -0.5
      critics: [..., "Twirling"]  # penalize unnecessary rotation

# AMCL (if using AMCL instead of slam_toolbox):
amcl:
  ros__parameters:
    robot_model_type: nav2_amcl::OmniMotionModel
```

### Test:
- [ ] DWB generates trajectories with `vy ≠ 0`
- [ ] Robot can perform oblique/crab obstacle avoidance
- [ ] Mode switch cooling prevents oscillation

---

## Phase 7: MPPI Comparison (Optional)

### New Config:
```yaml
# config/nav2_risk_mppi.yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "mppi_controller::MPPIController"
      # ... MPPI-specific params
```

### Test:
- [ ] A/B comparison: same start/goal, DWB vs MPPI
- [ ] Metrics: path length, smoothness, success rate, computation time

---

## Phase 8: Metrics & Analysis Package

### New Package: `ranger_nav_metrics` (Python)

#### `metrics_logger.py`
- Subscribes to all navigation topics
- Computes metrics online: success_rate, collision_rate, min_obstacle_dist, nav_time, path_length, replan_count, e_stop_count, vel_smoothness, local_failure_count, mode_switch_count
- Publishes `/navigation_metrics`
- Saves to CSV at end of run

#### `bag_analyzer.py`
- Offline analysis of rosbag files
- Computes same metrics from bag data
- Handles batch processing of experiment bags

#### `plot_results.py`
- Generates paper-ready plots:
  - Trajectory comparison (overlaid paths)
  - Obstacle distance over time
  - Velocity profiles
  - Risk level timeline
  - Replan event timeline
  - Mode switch timeline

---

## Phase 9: Experiment Automation

### New Script: `scripts/run_experiment.sh`

```bash
#!/bin/bash
# Usage: bash run_experiment.sh <experiment_name> <goal_x> <goal_y> <goal_yaw>
# Automates: start rosbag, send goal, wait for result, stop rosbag, save metrics
```

### New Config: `config/experiment_scenarios.yaml`
```yaml
scenarios:
  static_nav:
    goals: [[5.0, 0.0, 0.0], [0.0, 5.0, 1.57], [-5.0, 0.0, 3.14]]
    dynamic_obstacles: []
    
  narrow_passage:
    goals: [[8.0, 0.0, 0.0]]
    corridor_width: 1.2
    
  pedestrian_crossing:
    goals: [[10.0, 0.0, 0.0]]
    pedestrian_speed: 1.0
    crossing_angle: 90  # perpendicular
```

---

## Development Order Summary

| Order | Phase | Description | Estimated Effort |
|-------|-------|-------------|-----------------|
| 1 | P0 | Verify baseline, record baseline bags | 0.5 day |
| 2 | P1 | Fix footprint, safety limits in nav2_params.yaml | 0.5 day |
| 3 | P3 | obstacle_cluster_node.py | 1 day |
| 4 | P3 | d435i_obstacle_node.py | 1 day |
| 5 | P3 | sensor_fusion_node.py | 1 day |
| 6 | P2 | D435i costmap integration | 0.5 day |
| 7 | P4 | obstacle_tracker_node.py | 1.5 days |
| 8 | P4 | obstacle_predictor_node.py | 0.5 day |
| 9 | P4 | risk_evaluator_node.py | 1 day |
| 10 | P5 | replan_manager_node.py | 1.5 days |
| 11 | P6 | Enable Nav2 omni (vy_samples) | 0.5 day |
| 12 | P7 | MPPI config (optional) | 0.5 day |
| 13 | P8 | metrics_logger.py | 0.5 day |
| 14 | P8 | bag_analyzer.py | 1 day |
| 15 | P8 | plot_results.py | 1 day |
| 16 | P9 | Experiment scripts | 0.5 day |

**Total**: ~12 days of development, spread over 8-10 weeks with testing.
