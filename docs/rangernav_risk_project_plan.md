# RangerNav-Fusion: Project Plan

## 1. Project Name

**RangerNav-Fusion: Mode-Aware and Multi-Sensor Risk-Aware Navigation for Four-Wheel-Steering UGVs in Dynamic Environments**

## 2. Chinese Name

面向动态环境的四轮四转移动机器人运动模式感知与风险感知导航方法

## 3. Project Goals

1. **Platform-aware navigation**: Extend Nav2 to exploit Ranger Mini 2.0's four-wheel-steering capability (Ackermann, oblique, spin modes), rather than treating it as a differential-drive robot.
2. **Multi-sensor dynamic obstacle perception**: Lightweight MID360S + D435i fusion pipeline for real-time obstacle detection, tracking, and short-term prediction without deep learning.
3. **Risk-aware local planning**: Integrate TTC, minimum distance, sensor confidence, and motion-mode feasibility costs into local trajectory evaluation.
4. **Closed-loop replanning**: Detect local planner failures, robot stuck, and persistent dynamic obstruction to trigger global replanning and recovery.
5. **Real-robot validation**: Deploy on Ranger Mini 2.0 hardware, collect rosbags, and evaluate with quantitative metrics.

## 4. System Architecture

```
                    ┌──────────────────────────────────────┐
                    │            Nav2 Stack                │
                    │  ┌─────────┐  ┌───────────────────┐  │
                    │  │Planner  │  │Controller (DWB/   │  │
                    │  │(Navfn)  │  │MPPI + risk critic)│  │
                    │  └────┬────┘  └────────┬──────────┘  │
                    │       │                │              │
                    │  ┌────┴────────────────┴──────────┐   │
                    │  │     Costmap (global + local)    │   │
                    │  └────────────────┬───────────────┘   │
                    └──────────────────┼───────────────────┘
                                       │
            ┌──────────────────────────┼──────────────────┐
            │     RangerNav-Fusion     │                  │
            │                          │                  │
            │  ┌───────────────────────┴────────────┐     │
            │  │    Sensor Fusion & Perception      │     │
            │  │  ┌──────────┐    ┌──────────┐     │     │
            │  │  │ MID360S  │    │  D435i   │     │     │
            │  │  │ obstacle │    │ nearfield│     │     │
            │  │  │ detect   │    │ obstacle │     │     │
            │  │  └────┬─────┘    └────┬─────┘     │     │
            │  │       └──────┬────────┘           │     │
            │  │         ┌────┴────┐               │     │
            │  │         │ Fusion  │               │     │
            │  │         └────┬────┘               │     │
            │  │    ┌─────────┴─────────┐          │     │
            │  │    │ Tracking+Prediction│          │     │
            │  │    └─────────┬─────────┘          │     │
            │  └──────────────┼────────────────────┘     │
            │                 │                          │
            │  ┌──────────────┴────────────────────┐     │
            │  │    Risk Evaluation                │     │
            │  │    TTC + min_dist + confidence    │     │
            │  │    + path_dev + goal_progress     │     │
            │  │    + mode_feasibility             │     │
            │  └──────────────┬────────────────────┘     │
            │                 │                          │
            │  ┌──────────────┴────────────────────┐     │
            │  │    Replan Manager                  │     │
            │  │    Monitor nav state + risk        │     │
            │  │    Trigger clear/replan/recovery   │     │
            │  └──────────────┬────────────────────┘     │
            └─────────────────┼─────────────────────────┘
                              │
            ┌─────────────────┴─────────────────────────┐
            │     Mode-Aware Chassis Interface          │
            │  ┌──────────────────────────────────┐     │
            │  │   cmd_vel → mode dispatch        │     │
            │  │   Ackermann | Oblique | Spin     │     │
            │  │   + safety limits + cooling      │     │
            │  └──────────────────────────────────┘     │
            │                │                           │
            │         Ranger Mini 2.0                   │
            │         (CAN bus via ugv_sdk)              │
            └──────────────────────────────────────────┘
```

## 5. Software Modules

| Module | Package | Language | Description |
|--------|---------|----------|-------------|
| Chassis interface | `ranger_base` (existing, minimal changes) | C++ | Already has mode mapping; add cooling time, safety limits |
| Sensor drivers | `ranger_sensors.launch.py` (existing) | Python | Livox MID360S; add D435i launch |
| Dynamic obstacle detection | `ranger_dynamic_obstacle` (new) | Python | Clustering, tracking, prediction |
| Sensor fusion | `ranger_sensor_fusion` (new) | Python | MID360S + D435i obstacle fusion |
| Risk-aware controller | `nav2_controller` config (existing, reconfigured) | YAML | DWB/MPPI with risk critic |
| Replan manager | `ranger_replan_manager` (new) | Python | Failure detection + recovery trigger |
| Metrics & analysis | `ranger_nav_metrics` (new) | Python | Online metrics + offline analysis |

## 6. ROS2 Topic / Service / Action Design

### 6.1 New Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/obstacles_mid360` | `visualization_msgs/MarkerArray` | obstacle_cluster_node | MID360S detected obstacles |
| `/obstacles_d435i` | `visualization_msgs/MarkerArray` | d435i_obstacle_node | D435i near-field obstacles |
| `/fused_obstacles` | `visualization_msgs/MarkerArray` | sensor_fusion_node | Fused obstacle list |
| `/tracked_obstacles` | `visualization_msgs/MarkerArray` | obstacle_tracker_node | Tracked obstacles with IDs + velocity |
| `/predicted_obstacles` | `visualization_msgs/MarkerArray` | obstacle_predictor_node | Obstacles with predicted trajectories |
| `/risk_markers` | `visualization_msgs/MarkerArray` | risk_evaluator_node | Risk visualization (TTC, danger zones) |
| `/near_field_safety_zone` | `visualization_msgs/Marker` | d435i_obstacle_node | Near-field safety zone polygon |
| `/navigation_metrics` | `ranger_msgs/NavigationMetrics` | metrics_logger.py | Online navigation metrics |
| `/replan_event` | `std_msgs/String` | replan_manager_node | Replan trigger event log |

### 6.2 New Services

| Service | Type | Server | Description |
|---------|------|--------|-------------|
| `/replan_manager/trigger_replan` | `std_srvs/Trigger` | replan_manager_node | Manually trigger replan |
| `/replan_manager/reset_state` | `std_srvs/Trigger` | replan_manager_node | Reset failure counters |
| `/sensor_fusion/reset` | `std_srvs/Trigger` | sensor_fusion_node | Clear fusion state |

### 6.3 Subscribed Topics (new modules)

| Subscriber | Topic | Purpose |
|------------|-------|---------|
| obstacle_cluster_node | `/livox/lidar` | Raw MID360S pointcloud |
| obstacle_cluster_node | `/camera/depth/color/points` | D435i depth pointcloud |
| obstacle_tracker_node | `/obstacles_mid360`, `/obstacles_d435i` | Track obstacles across frames |
| replan_manager_node | `/plan`, `/local_plan`, `/cmd_vel`, `/risk_markers`, `/odom`, `/system_state` | Monitor nav health |
| metrics_logger | `/odom`, `/plan`, `/cmd_vel`, `/risk_markers` | Log metrics |

## 7. Nav2 Integration

### 7.1 DWB/MPPI Configuration

- **Enable lateral velocity**: `max_vel_y > 0`, `vy_samples > 0`
- **Add custom risk critic**: or modify `ObstacleFootprint` to accept external risk input
- **Costmap integration**: costmap obstacle_layer from `/scan` + fused obstacles via custom layer

### 7.2 Localization

- **Option A**: Keep `slam_toolbox` localization (current)
- **Option B**: Add `nav2_amcl` with `OmniMotionModel` for comparison
- **Recommendation**: Support both for ablation experiments

### 7.3 Behavior Tree

- Extend default BT to include risk-check condition
- Add `ClearCostmapOnStuck` custom behavior
- Keep all existing behaviors (spin, backup, wait)

## 8. Real-Robot Experiment Flow

1. **Pre-flight**:
   - CAN bringup (`can_bringup.sh`)
   - Livox network config
   - Start `ranger_full.launch.py mode:=nav`
   - Verify TF tree: `ros2 run tf2_tools view_frames`
   - Verify `/scan` rate: `ros2 topic hz /scan`

2. **Experiment run**:
   - Start `rosbag record` with required topics
   - Set initial pose in RViz
   - Send navigation goal
   - Monitor risk markers, replan events
   - Record quantitative metrics

3. **Post-run**:
   - Stop rosbag
   - Run `bag_analyzer.py`
   - Generate plots with `plot_results.py`

## 9. Risk & Safety Notes

1. **Hardware safety**:
   - Always keep remote control within reach
   - Test mode switching at low speed (< 0.3 m/s) first
   - Set `max_vel_x` to 1.0 m/s maximum during experiments

2. **Emergency stop**:
   - Remote control kill switch
   - `/cmd_vel` zero on error state
   - Low battery auto-stop at 10% SOC

3. **CAN bus**:
   - CAN cable must be secured during experiments
   - Monitor `/system_state.error_code` for CAN communication errors

4. **Sensor safety**:
   - MID360S has 0.5m blind zone — D435i fills this gap
   - D435i max range ~4m reliable (daylight), ~6m (indoor)
   - Ensure both sensors are rigidly mounted

## 10. Milestone Plan

| Milestone | Deliverable | Target |
|-----------|------------|--------|
| M1: MVP-1 | MID360S only → /scan → Nav2 baseline nav works | Week 1 |
| M2: MVP-2 | D435i near-field → local costmap works | Week 2 |
| M3: MVP-3 | Fused obstacles → RViz Marker visualization | Week 3 |
| M4: MVP-4 | Dynamic obstacle tracking + prediction → TTC risk markers | Week 4 |
| M5: MVP-5 | Risk markers + Nav2 state → replan trigger works | Week 5 |
| M6: DWB omni | Enable vy in DWB, test oblique avoidance | Week 6 |
| M7: MPPI eval | MPPI comparison with DWB | Week 7 |
| M8: Full system | All modules integrated, real-robot testing | Week 8 |
| M9: Experiment | Run all 15 experiments, collect data | Week 9-10 |
| M10: Paper | Complete draft, analyze results | Week 11-12 |
