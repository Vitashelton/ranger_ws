# Method Design — RangerNav-Fusion

## A. Mode-Aware Chassis Interface

### A.1 Why Ranger Mini 2.0 Cannot Be Treated as Differential Drive

Ranger Mini 2.0 is a four-wheel-steering (4WS) UGV. Each wheel has independent steering angle control. The CAN protocol supports the following discrete motion modes (`ranger_interface.hpp:53-59`):

| Mode | CAN enum | Description |
|------|----------|-------------|
| Dual Ackermann | 0 | Front and rear wheels steer opposite for tight turns. Like a car but with rear counter-steering. |
| Parallel | 1 | All wheels steer in the same direction. Robot moves diagonally/crab-wise. |
| Spinning | 2 | Wheels steer tangent to a circle. Robot rotates in place. |
| Park | 3 | Wheels lock, motors disengage. |
| Side Slip | 4 | Wheels steer 90°. Robot moves purely laterally. |

Key constraint: **The 4WS UGV cannot execute arbitrary holonomic vx+vy+wz simultaneously.** A standard omnidirectional base (e.g., mecanum wheel) can independently control all three degrees. Ranger Mini 2.0 must select exactly one mode per command cycle.

### A.2 cmd_vel → Motion Mode Mapping

The current implementation in `ranger_messenger.cpp:410-516` already has the correct logic. We formalize it:

```
Input: cmd_vel = (vx, vy, wz)

1. If |vx| < ε AND |vy| < ε AND |wz| < ε:
   → STOP (zero command)

2. If |vy| > ε:
   → OBLIQUE MODE (Parallel/SideSlip)
   - speed = hypot(vx, vy)
   - steer = atan2(vy, vx), clamped to ±max_steer_angle_parallel
   - wz is DROPPED (warning throttled at 2s)
   
3. If |vx| < ε AND |wz| > ε (pure rotation):
   → SPIN MODE
   - angular = clamp(wz, ±max_angular_speed)
   
4. If |vx| > ε AND |wz| > ε:
   - radius = |vx| / |wz|
   - If radius < min_turn_radius:
     → SPIN MODE (turn radius too tight for Ackermann)
   - Else:
     → ACKERMANN MODE
     - linear = clamp(vx, ±max_linear_speed)
     - steer = atan(wheelbase/2 / radius), clamped to ±max_steer_angle_ackermann

5. If |vx| > ε AND |wz| < ε:
   → ACKERMANN MODE (straight line)
```

### A.3 Safety Limits

| Parameter | Value | Source |
|-----------|-------|--------|
| `max_linear_speed` | 1.5 m/s | `ranger_params.hpp:55` (RangerMiniV2Params) |
| `max_angular_speed` | 4.8 rad/s | `ranger_params.hpp:56` |
| `max_steer_angle_parallel` | 1.57 rad (90°) | `ranger_params.hpp:60` |
| `max_steer_angle_ackermann` | 0.698 rad (40°) | `ranger_params.hpp:63` |
| `min_turn_radius` | 0.476 m | `ranger_params.hpp:62` |

Additional safety limits to add:

| Parameter | Recommended Value | Rationale |
|-----------|------------------|-----------|
| `max_vel_y` (for Nav2) | 0.3 m/s | Lateral movement is less stable; limit for safety |
| `mode_switch_cooldown` | 0.5 s | Prevent rapid mode oscillation wearing steering actuators |
| `emergency_stop_accel` | 2.0 m/s² | Maximum deceleration on e-stop |
| `low_battery_stop_soc` | 0.10 | Auto-stop below 10% SOC |
| `error_state_stop` | true | Stop immediately if system_state.error_code ≠ 0 |

### A.4 Mode Switch Cooling Time

Added to `TwistCmdCallback`:
```cpp
// In RangerROSMessenger class:
rclcpp::Time last_mode_switch_time_;
static constexpr double kModeSwitchCooldown = 0.5;  // seconds

// In TwistCmdCallback, before SetMotionMode:
if ((current_time_ - last_mode_switch_time_).seconds() < kModeSwitchCooldown) {
    // Keep previous mode if within cooldown
    return;  // or: execute command in current mode but don't switch
}
last_mode_switch_time_ = current_time_;
```

---

## B. Dynamic Obstacle Perception and Prediction

### B.1 Design Principles

- **No deep learning**: All methods are geometric/statistical
- **Real-time**: Target 10 Hz on Jetson Orin
- **Lightweight**: Euclidean clustering + Kalman filter tracking
- **Deterministic**: Reproducible results for scientific evaluation

### B.2 MID360S Processing Pipeline

```
/livox/lidar (PointCloud2, 360° FOV, 10 Hz)
    │
    ▼
[ROI Filter]
    - Remove points above 2.0m (ceiling) and below 0.1m (ground)
    - Remove points beyond 15m (range limit)
    - Keep points in front 270° (trim rear self-occlusion)
    │
    ▼
[Ground Removal]
    - RANSAC plane fitting (threshold 0.05m)
    - Remove inlier points (ground)
    │
    ▼
[Voxel Downsample]
    - Leaf size: 0.1m
    - Reduces point count ~10x for clustering
    │
    ▼
[Euclidean Clustering]
    - Cluster tolerance: 0.15m
    - Min cluster size: 5 points
    - Max cluster size: 5000 points
    │
    ▼
[Bounding Box Extraction]
    - 2D bounding box (centroid, width, length)
    - Centroid (x, y) in base_link frame
    │
    ▼
/obstacles_mid360 (MarkerArray, Cylinder markers)
```

### B.3 D435i Processing Pipeline

```
/camera/depth/color/points (PointCloud2, ~87°×58° FOV, 6-15 Hz)
    │
    ▼
[ROI Filter]
    - Range: 0.2m to 4.0m (near-field only)
    - Height: 0.0m to 1.5m (low obstacles only)
    │
    ▼
[Depth-based Clustering]
    - Cluster tolerance: 0.08m (denser than LiDAR)
    - Min cluster size: 10 points
    │
    ▼
[Low Obstacle Flag]
    - If cluster max_z < 0.3m → flag as "low obstacle"
    │
    ▼
/obstacles_d435i (MarkerArray, Cube markers)
/near_field_safety_zone (Marker, Polygon in front of robot)
```

### B.4 Multi-Frame Tracking (Kalman Filter)

State vector per obstacle:
```
x = [px, py, vx, vy]^T
```

Constant velocity model:
```
x_k = F x_{k-1} + w
F = [1 0 dt 0
     0 1 0  dt
     0 0 1  0
     0 0 0  1]
```

Measurement:
```
z = [px_meas, py_meas]^T
H = [1 0 0 0
     0 1 0 0]
```

Process noise Q and measurement noise R tuned per sensor:
- MID360S: σ_meas = 0.03m (LiDAR accuracy)
- D435i: σ_meas = 0.02m at 1m, 0.05m at 3m (depth-dependent)

Data association: Hungarian algorithm with cost = Euclidean distance + size similarity.

Track management:
- **Birth**: 3 consecutive detections
- **Death**: 5 consecutive misses
- **Confirmed**: track exists > 10 frames → velocity estimate reliable

### B.5 Short-Term Trajectory Prediction

For each confirmed track with velocity `v = (vx, vy)`:
```
predicted_position(t) = current_position + v × t
predicted_uncertainty(t) = σ² × t²  (growing ellipse)
```

Prediction horizon: 2.0s, at 0.2s steps. Output as MarkerArray of LINE_STRIP markers.

---

## C. Risk-Aware Local Trajectory Evaluation

### C.1 Cost Function Design

The total cost for a trajectory candidate τ is:

```
J(τ) = w_goal · J_goal(τ)
     + w_path · J_path(τ)
     + w_static · J_static(τ)
     + w_dyn · J_dynamic(τ)
     + w_ttc · J_ttc(τ)
     + w_smooth · J_smooth(τ)
     + w_mode · J_mode(τ)
     + w_conf · J_confidence(τ)
```

### C.2 Individual Cost Terms

**Goal Progress (J_goal)**:
```
J_goal = ||τ(T) - goal||₂
```
Distance from trajectory end to goal. T = sim_time.

**Path Deviation (J_path)**:
```
J_path = max_{t∈[0,T]} min_{p∈global_path} ||τ(t) - p||₂
```
Maximum lateral deviation from global path.

**Static Obstacle (J_static)**:
```
J_static = Σ_{t} Σ_{o∈static} max(0, d_safe - d(τ(t), o))²
```
Quadratic penalty when closer than `d_safe = inflation_radius + robot_radius`.

**Dynamic Obstacle (J_dynamic)**:
```
J_dynamic = Σ_{t} Σ_{o∈dynamic} max(0, d_dyn - d(τ(t), pred_o(t)))²
```
where `d_dyn = 1.0m` (larger safety margin for moving obstacles). Uses **predicted** obstacle positions at each time step.

**Time-To-Collision (J_ttc)**:
```
TTC(τ, o) = d / max(-v_rel, ε)   where v_rel is closing speed

J_ttc = Σ_{t} Σ_{o} {
    1.0 / (TTC(τ(t), o) + ε),  if TTC < T_thresh
    0,                          otherwise
}
```
where `T_thresh = 3.0s`. Penalty spikes as TTC approaches zero.

**Control Smoothness (J_smooth)**:
```
J_smooth = ||v(τ_k) - v(τ_{k-1})||₂² + ||ω(τ_k) - ω(τ_{k-1})||₂²
```
Penalizes abrupt velocity/steering changes between consecutive commands.

**Motion Mode Cost (J_mode)**:
```
J_mode = {
    0,      if mode = current_mode (no switch)
    w_switch, if mode ≠ current_mode (penalize switching)
}
```
Encourages maintaining the current mode to reduce actuator wear.

**Sensor Confidence (J_confidence)**:
```
J_confidence = Σ_{t} Σ_{o} (1 - confidence_o) · penalty(d)
```
Obstacles with low confidence are still considered but with reduced weight. `confidence_o` comes from the sensor fusion module.

### C.3 Weight Configuration

| Weight | Symbol | Default | Tuning Range |
|--------|--------|---------|-------------|
| Goal | w_goal | 1.0 | 0.5–2.0 |
| Path | w_path | 10.0 | 5.0–30.0 |
| Static | w_static | 20.0 | 10.0–50.0 |
| Dynamic | w_dyn | 30.0 | 15.0–60.0 |
| TTC | w_ttc | 40.0 | 20.0–80.0 |
| Smooth | w_smooth | 2.0 | 1.0–5.0 |
| Mode | w_mode | 1.0 | 0.5–3.0 |
| Confidence | w_conf | 5.0 | 2.0–10.0 |

### C.4 Integration with DWB

Option A (simple): Add custom critics as DWB plugin:
- `RiskObstacleCritic` — combines J_dynamic + J_ttc + J_confidence
- `ModeSwitchCritic` — implements J_mode

Option B (recommended for paper): Replace DWB critics with a Python `RiskEvaluator` node that:
1. Receives DWB trajectory samples (via `/local_plan` or custom topic)
2. Evaluates risk costs using fused obstacles + predictions
3. Publishes risk-modified trajectory scores
4. Can work with both DWB and MPPI

### C.5 Near-Field Safety Zone

D435i provides a "hard safety zone" in front of the robot (0.2m to 1.0m). Any obstacle detected in this zone triggers:
- Scale DWB velocity to 0.2× (slowdown)
- If obstacle < 0.3m: immediate stop
- Publish `/near_field_safety_zone` marker for visualization

---

## D. Replanning Trigger and Recovery

### D.1 Trigger Conditions

| Condition | Detection Method | Threshold |
|-----------|-----------------|-----------|
| Local planner failure | Monitor `/plan` + `/local_plan` mismatch or controller_server feedback | 3 consecutive failures |
| Robot stuck | `/odom` velocity < 0.05 m/s for > 5s while goal not reached | velocity < 0.05 m/s, duration > 5s |
| Global path blocked | Check if global path crosses fused obstacle with high confidence for > 3s | persistent obstruction > 3s |
| High TTC risk | `min(TTC) < 1.0s` for > 2s | TTC < 1.0s, > 2s |
| Excessive emergency stops | Count e-stop events in sliding 30s window | > 3 stops in 30s |

### D.2 Recovery Actions (Priority Order)

1. **Slow down**: Scale `/cmd_vel` by 0.5 for one planning cycle (least invasive)
2. **Clear local costmap**: `rosservice call /local_costmap/clear_entirely` — removes stale obstacles
3. **Clear global costmap**: For persistent blocked path
4. **Replan**: `rosservice call /planner_server/compute_path_to_pose` with same goal
5. **Spin adjust**: Small-angle rotation (±30°) to find new local path
6. **Wait**: 2s pause for dynamic obstacles to clear
7. **Safety stop**: If all above fail or TTC < 0.5s, cancel navigation goal and stop

### D.3 Implementation

`ranger_replan_manager` node:
- Subscribes to: `/odom`, `/plan`, `/local_plan`, `/cmd_vel`, `/risk_markers`, `/system_state`, `/goal_pose`
- Timer at 10 Hz: evaluates all trigger conditions
- Publishes `/replan_event` (String) for logging
- Calls Nav2 services programmatically
