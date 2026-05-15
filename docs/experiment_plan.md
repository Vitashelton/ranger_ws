# Experiment Plan — RangerNav-Fusion

## Overview

All experiments run on Ranger Mini 2.0 hardware with MID360S LiDAR and D435i RGB-D camera. Data recorded via rosbag for offline analysis.

---

## Experiment 1: Static Map Navigation (Baseline)

### Setup
- **Scene**: Indoor corridor or open lab space, static furniture only
- **Map**: Pre-built 2D occupancy grid (from slam_toolbox or FAST-LIO2→pcd_to_2d_map)
- **Start**: (0, 0, 0°)
- **Goals**: (5, 0, 0°), (0, 5, 90°), (-5, 0, 180°)
- **Dynamic obstacles**: None

### Methods Compared
1. DWB differential (max_vel_y=0, vy_samples=0) — current config
2. DWB omni (max_vel_y=0.3, vy_samples=11)

### Metrics
- Success rate, path length, navigation time, velocity smoothness, mode_switch_count, lateral_motion_usage_ratio

### Expected Result
- Both methods succeed
- No significant difference (no lateral obstacles to avoid)
- Establishes baseline performance

### rosbag Topics
```
/odom /cmd_vel /plan /local_plan /scan /tf /tf_static /goal_pose 
/system_state /motion_state /battery_state
```

### Possible Failure
- Localization drift over long distances
- Goal tolerance not reached (adjust xy_goal_tolerance)

---

## Experiment 2: Narrow Passage Obstacle Avoidance

### Setup
- **Scene**: Corridor with two obstacles forming a 1.2m gap (narrower than robot + inflation)
- **Start**: (0, 0, 0°), facing the gap
- **Goal**: (6, 0, 0°), beyond the gap
- **Dynamic obstacles**: None

### Methods Compared
1. DWB differential — must align straight, no lateral avoidance
2. DWB omni — can crab through gap

### Metrics
- Success rate, minimum obstacle distance, path length, lateral_motion_usage_ratio, velocity smoothness

### Expected Result
- DWB omni uses lateral motion to navigate tight gaps more efficiently
- DWB differential may oscillate or fail if gap requires non-axial approach

### rosbag Topics
Same as Experiment 1.

### Possible Failure
- Gap too narrow for either config → widen gap or reduce inflation_radius
- DWB omni oscillates between Ackermann and oblique → tune mode_switch_cooldown

---

## Experiment 3: Pedestrian Crossing (Dynamic Obstacle)

### Setup
- **Scene**: Open space with a pedestrian walking perpendicular to robot path
- **Start**: (0, 0, 0°)
- **Goal**: (8, 0, 0°)
- **Dynamic obstacle**: Human walks from (4, -3) to (4, 3) at ~1.0 m/s, crossing the path
- **Timing**: Pedestrian reaches crossing point when robot is ~3m away

### Methods Compared
1. DWB differential (no tracking, no prediction)
2. DWB with risk critic (tracking + TTC)
3. DWB omni + risk critic

### Metrics
- Success rate, collision rate, minimum obstacle distance, TTC_min, emergency stop count, navigation time, replan_count

### Expected Result
- Without tracking: robot may not react in time or brake late
- With tracking + TTC: early slowdown or lateral avoidance
- Omni mode enables lateral dodge alongside slowdown

### rosbag Topics
Same + `/tracked_obstacles` `/predicted_obstacles` `/risk_markers` `/replan_event`

### Possible Failure
- Tracking loses dynamic obstacle (occlusion, sensor FOV limit)
- Speed estimation inaccurate → TTC wrong → false emergency stop
- Pedestrian movement not constant-velocity → prediction wrong

---

## Experiment 4: Temporary Obstacle Blocking Global Path

### Setup
- **Scene**: Person stands in front of robot on global path for 5–10s, then walks away
- **Start**: (0, 0, 0°)
- **Goal**: (10, 0, 0°)
- **Dynamic obstacle**: Human stands at (3, 0) for 5–10s, then walks away

### Methods Compared
1. DWB differential (without replan manager)
2. DWB differential + replan manager
3. DWB omni + risk critic + replan manager (full system)

### Metrics
- Replan count, local_planner_failure_count, navigation time, time_stuck, emergency_stop_count

### Expected Result
- Without replan manager: robot waits forever or fails after timeout
- With replan manager: detects stuck, clears costmap, replans (or waits for obstacle to clear)
- Full system: risk evaluator detects persistent TTC → triggers replan

### rosbag Topics
Same + `/replan_event` `/navigation_metrics`

### Possible Failure
- Replan manager triggers too aggressively (false positive stuck detection)
- Clear costmap removes valid static obstacles → collision risk

---

## Experiment 5: Ackermann/Diff Config vs Omni Config (Ablation)

### Setup
- **Scene**: Mixed static obstacles requiring lateral deviation from global path
- **Start**: (0, 0, 0°)
- **Goal**: (6, 2, 45°)
- **Obstacles**: 3 static obstacles positioned to force lateral avoidance

### Methods Compared
1. DWB differential (max_vel_y=0) — must maneuver via turns
2. DWB omni (max_vel_y=0.3) — can crab laterally

### Metrics
- Path length, navigation time, velocity smoothness, mode_switch_count, lateral_motion_usage_ratio, minimum obstacle distance

### Expected Result
- Omni config produces shorter, smoother paths by using lateral motion
- Ackermann-only config requires more turning (zigzag pattern)

### rosbag Topics
Same as Experiment 1 + mode_switch_count from `/motion_state`

---

## Experiment 6: MPPI vs DWB (Optional)

### Setup
- **Scene**: Same as Experiment 3 (pedestrian crossing)
- **Methods**: DWB omni vs MPPI
- **Metrics**: Same as Experiment 3 + computation time, trajectory smoothness

### Expected Result
- [TBD] MPPI may produce smoother trajectories but potentially higher computation
- [TBD] DWB is mature, MPPI may need tuning for Ranger's dynamics

---

## Experiment 7: With/Without Risk Critic (Ablation)

### Setup
- **Scene**: Experiment 3 (pedestrian crossing)
- **Methods**:
  1. DWB omni without risk critic (standard ObstacleFootprint only)
  2. DWB omni with TTC risk critic
  3. DWB omni with full risk critic (TTC + confidence + mode)

### Metrics
- collision_rate, min_obstacle_distance, TTC_min, emergency_stop_count, navigation_time

### Expected Result
- Full risk critic achieves lowest collision rate and highest min distance
- Adding confidence weighting reduces false positive risk from D435i noise

---

## Experiment 8: With/Without Replan Trigger (Ablation)

### Setup
- **Scene**: Combined Experiment 3 + 4 (pedestrian crossing + temporary blockage)
- **Methods**:
  1. No replan manager
  2. Replan manager enabled

### Metrics
- Success rate, navigation time, replan_count, time_stuck, emergency_stop_count

---

## Sensor Ablation Experiments

### Experiment S1: MID360S Only
- D435i disabled. Only `/scan` → costmap. No near-field safety zone.
- **Tests**: All Experiments 1–4
- **Expected**: Good for mid-range obstacles. Fails on low obstacles and in blind zone (< 0.5m).

### Experiment S2: D435i Only
- MID360S disabled. Only D435i for perception.
- **Tests**: Experiment 1 only (static map not possible without LiDAR SLAM)
- **Expected**: Near-field obstacle detection works. Fails at ranges > 4m and in poor lighting.

### Experiment S3: MID360S + D435i Fusion
- Both sensors active, fusion enabled.
- **Tests**: All Experiments 1–4
- **Expected**: Best performance overall. D435i fills blind zone. MID360S provides long-range perception.

### Experiment S4: Fusion Without Prediction
- Fusion enabled, but obstacle_predictor_node disabled.
- **Tests**: Experiment 3 (pedestrian crossing)
- **Expected**: Robot reacts to current obstacle position only, not predicted. Later reaction → closer encounters.

### Experiment S5: Fusion With Prediction
- Full pipeline: fusion + tracking + prediction.
- **Tests**: Experiment 3
- **Expected**: Earlier reaction to approaching obstacles. Higher TTC_min.

### Experiment S6: Fusion With Risk-Aware Planner
- Complete system.
- **Tests**: All Experiments
- **Expected**: Best overall metrics across all scenarios.

### Sensor Ablation Metrics (Additional)

| Metric | Definition |
|--------|-----------|
| False positive obstacle count | Detected obstacle where none exists |
| False negative obstacle count | Missed obstacle that exists (determined from ground truth annotation) |
| Sensor dropout count | Number of times a sensor topic stops publishing for > 1s |
| Computation time (ms) | Per-node processing time |
| MID360S detection rate | Detected / total obstacles in MID360S FOV |
| D435i detection rate | Detected / total obstacles in D435i FOV |
| Fusion improvement ratio | Success rate(fusion) / success rate(best single sensor) |

---

## Evaluation Metrics Summary

| # | Metric | Type | Unit |
|---|--------|------|------|
| 1 | Success rate | Primary | % |
| 2 | Collision rate | Primary | % |
| 3 | Minimum obstacle distance | Primary | m |
| 4 | Average navigation time | Secondary | s |
| 5 | Path length | Secondary | m |
| 6 | Replan count | Secondary | count |
| 7 | Emergency stop count | Secondary | count |
| 8 | Velocity smoothness | Secondary | Σ‖Δv‖² |
| 9 | Local planner failure count | Secondary | count |
| 10 | Computation time (per module) | Secondary | ms |
| 11 | Mode switch count | Secondary | count |
| 12 | Lateral motion usage ratio | Secondary | % |
| 13 | TTC minimum | Primary | s |
| 14 | False positive obstacle count | Sensor ablation | count |
| 15 | False negative obstacle count | Sensor ablation | count |
| 16 | Sensor dropout cases | Sensor ablation | count |

---

## rosbag Recording Command

```bash
ros2 bag record \
  /odom /cmd_vel /plan /local_plan /scan /tf /tf_static \
  /goal_pose /system_state /motion_state /battery_state \
  /obstacles_mid360 /obstacles_d435i /fused_obstacles \
  /tracked_obstacles /predicted_obstacles /risk_markers \
  /near_field_safety_zone /replan_event /navigation_metrics \
  -o experiment_<name>_<method>_<trial>
```

---

## Ground Truth Annotation

For quantitative evaluation of dynamic obstacle detection:

1. Record rosbag with all topics
2. Manually annotate dynamic obstacle positions every 0.5s (in RViz or Foxglove)
3. Save annotations as CSV: `timestamp, obstacle_id, px, py, vx, vy, width, length`
4. `bag_analyzer.py` loads annotations and computes detection errors

---

## Statistical Reporting

Report for each experiment:
- Mean ± std for each metric across 5 trials
- Per-method comparison (bar chart or box plot)
- Trajectory overlay plot (all trials on one map)
- Obstacle distance vs time plot
- Risk level timeline (heatmap)
