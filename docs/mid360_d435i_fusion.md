# MID360S + D435i Multi-Sensor Fusion Design

## 1. Sensor Role Partition

| Sensor | FOV | Range | Strengths | Weaknesses | Role |
|--------|-----|-------|-----------|------------|------|
| MID360S | 360° H × 59° V | 0.5–40m | Full surround, long range, 3D, SLAM-capable | 0.5m blind zone, tilted mount misses near-ground | **Primary**: SLAM, localization, mid/long-range obstacle detection |
| D435i | 87° H × 58° V | 0.2–4m | Near-field accuracy, low obstacles, RGB texture | Narrow FOV, range limited, IR degraded outdoors | **Secondary**: Near-field blind zone coverage, low obstacle detection |

## 2. Fusion Architecture

```
┌─────────────────────────────────────────────────┐
│                Sensor Layer                      │
│  ┌──────────────┐         ┌──────────────┐      │
│  │  MID360S     │         │  D435i       │      │
│  │  /livox/lidar│         │  /camera/    │      │
│  │              │         │  depth/color/│      │
│  │              │         │  points      │      │
│  └──────┬───────┘         └──────┬───────┘      │
│         │                        │               │
├─────────┼────────────────────────┼───────────────┤
│         ▼                        ▼               │
│  ┌──────────────┐         ┌──────────────┐      │
│  │ ROI Filter   │         │ ROI Filter   │      │
│  │ Voxel Down   │         │ Range: 0.2-  │      │
│  │ Ground Remove│         │  4.0m        │      │
│  └──────┬───────┘         │ Height: 0-   │      │
│         │                 │  1.5m        │      │
│         ▼                 └──────┬───────┘      │
│  ┌──────────────┐                │               │
│  │ Euclidean    │                ▼               │
│  │ Clustering   │         ┌──────────────┐      │
│  │ tol=0.15m    │         │ Depth-based  │      │
│  └──────┬───────┘         │ Clustering   │      │
│         │                 │ tol=0.08m    │      │
│         ▼                 └──────┬───────┘      │
│  /obstacles_mid360                │               │
│  (MarkerArray)                    ▼               │
│                          /obstacles_d435i         │
│                          (MarkerArray)            │
│                          /near_field_safety_zone  │
│                          (Marker)                 │
│         │                        │               │
├─────────┼────────────────────────┼───────────────┤
│         └──────────┬─────────────┘               │
│                    ▼                              │
│  ┌─────────────────────────────────────┐         │
│  │        Sensor Fusion Node           │         │
│  │                                     │         │
│  │  1. Approximate time sync           │         │
│  │  2. Spatial association             │         │
│  │  3. Obstacle merging                │         │
│  │  4. Confidence assignment           │         │
│  └────────────────┬────────────────────┘         │
│                   ▼                               │
│  /fused_obstacles (MarkerArray + confidence)      │
│                   │                               │
│                   ▼                               │
│  ┌─────────────────────────────────────┐         │
│  │        Tracker + Predictor          │         │
│  │                                     │         │
│  │  Kalman Filter (px, py, vx, vy)     │         │
│  │  Hungarian data association         │         │
│  │  Constant-velocity prediction       │         │
│  └────────────────┬────────────────────┘         │
│                   ▼                               │
│  /tracked_obstacles + /predicted_obstacles        │
│                   │                               │
│                   ▼                               │
│  ┌─────────────────────────────────────┐         │
│  │        Risk Evaluator               │         │
│  │                                     │         │
│  │  TTC, min_dist, confidence-weighted │         │
│  └────────────────┬────────────────────┘         │
│                   ▼                               │
│  /risk_markers (MarkerArray)                      │
└─────────────────────────────────────────────────┘
```

## 3. Obstacle Association Algorithm

```
Input: obstacles_mid360[], obstacles_d435i[], timestamp
Output: matched_pairs[], unmatched_mid360[], unmatched_d435i[]

1. Transform all obstacles to common frame (base_link)

2. Build cost matrix C[i][j] = Euclidean distance between
   mid360_obs[i].center and d435i_obs[j].center

3. If mid360_obs[i] and d435i_obs[j] are both in overlapping FOV:
      C[i][j] += size_similarity_cost:
         = |mid360_obs[i].width - d435i_obs[j].width|
         + |mid360_obs[i].length - d435i_obs[j].length|

4. Hungarian algorithm: find optimal assignment minimizing total cost

5. For each assigned pair:
      if C[i][j] < association_threshold (0.5m):
          → merge pair
      else:
          → keep as separate obstacles

6. Association threshold:
   - Same FOV region: 0.5m
   - Disjoint FOV region: N/A (can't associate)
```

## 4. Obstacle Merging

When an obstacle is detected by both sensors:
```
merged.center = (mid360.center * w_mid + d435i.center * w_d435i) / (w_mid + w_d435i)

merged.width  = max(mid360.width, d435i.width)   # conservative: use larger
merged.length = max(mid360.length, d435i.length)

merged.confidence = 1.0 - (1 - c_mid) * (1 - c_d435i)
# 0.8 + 0.6 → 0.92 (independent sensor assumption)
# Capped at 0.95 max
```

## 5. Confidence Model

| Condition | Confidence | Description |
|-----------|-----------|-------------|
| MID360S obstacle, range ≤ 10m | 0.85 | LiDAR high confidence in mid-range |
| MID360S obstacle, 10m < range ≤ 20m | 0.70 | Range falloff |
| MID360S obstacle, range > 20m | 0.50 | Very far, sparse points |
| D435i obstacle, range ≤ 1m | 0.80 | Depth camera best at close range |
| D435i obstacle, 1m < range ≤ 3m | 0.60 | Moderate depth noise |
| D435i obstacle, range > 3m | 0.40 | High depth noise, low resolution |
| Both sensors agree | min(0.95, 1-(1-c_mid)*(1-c_d435i)) | Independent confirmation |
| Low obstacle (z < 0.3m), D435i only | +0.10 bonus | D435i is the only sensor that can see these |
| Persistent obstacle (> 10 frames tracked) | +0.05 bonus | Temporal consistency bonus |

## 6. FOV Overlap Region

The overlapping FOV region (where both sensors see):
- Azimuth: approximately [-43°, +43°] (D435i 87° H-FOV centered forward)
- Range: 0.5m to 4.0m (MID360S min range to D435i max range)

Outside this region, only one sensor is active:
- Behind/near robot: MID360S only
- Far forward (> 4m): MID360S only
- Very close (< 0.5m): D435i only (MID360S blind zone)

## 7. Costmap Integration

### Global Costmap
```
MID360S → /scan → obstacle_layer (marking + clearing)
D435i → NOT included (avoid noise in static map)
```

### Local Costmap
```
MID360S → /scan → voxel_layer (marking + clearing)
D435i → /d435i/scan → voxel_layer (marking + clearing, max_range: 4.0)
/near_field_safety_zone → NOT in costmap (used by risk evaluator, not costmap)
```

### Why D435i Not in Global Costmap
1. D435i FOV is narrow (87°), covering only forward direction
2. Depth noise increases with range; at 4m, errors can be > 5cm
3. IR depth is unreliable outdoors
4. The global costmap should represent a reliable, static world model
5. D435i near-field detection is transient — better suited for local costmap and risk evaluation

## 8. Near-Field Safety Zone Implementation

```python
# d435i_obstacle_node.py

safety_zone_polygon = Polygon([
    (0.1, -0.3),   # front-left of robot
    (0.1, 0.3),    # front-right
    (1.0, 0.4),    # 1m forward, slightly wider
    (1.0, -0.4),   # 1m forward
])

# Check each obstacle in /obstacles_d435i
for obs in obstacles_d435i:
    if obs.center in safety_zone_polygon:
        zone_level = "WARNING" if obs.range > 0.5 else "CRITICAL"
        if zone_level == "CRITICAL":
            publish_emergency_slowdown()
```

## 9. Configuration File Structure

### `config/sensor_fusion.yaml`
```yaml
sensor_fusion:
  ros__parameters:
    # Association
    association_max_dist: 0.5
    association_size_weight: 0.2
    max_timestamp_diff: 0.1          # max 100ms time gap for sync

    # Confidence
    mid360_base_confidence: 0.85
    mid360_range_decay_start: 10.0   # start decaying at 10m
    mid360_range_decay_end: 20.0     # min confidence at 20m
    d435i_base_confidence: 0.8
    d435i_range_decay_start: 1.0
    d435i_range_decay_end: 3.0
    dual_detection_bonus: 0.95
    low_obstacle_bonus: 0.10
    temporal_consistency_bonus: 0.05
    min_confidence_threshold: 0.3    # discard obstacles below this

    # Near-field safety zone
    safety_zone_x_min: 0.2
    safety_zone_x_max: 1.0
    safety_zone_y_half_width: 0.4
    safety_critical_range: 0.3       # immediate stop below this

    # Visualization
    publish_markers: true
    mid360_marker_color: [0.0, 1.0, 0.0]   # green
    d435i_marker_color: [0.0, 0.0, 1.0]    # blue
    fused_marker_color: [0.0, 1.0, 1.0]    # cyan
    risk_low_color: [0.0, 1.0, 0.0]        # green
    risk_medium_color: [1.0, 1.0, 0.0]     # yellow
    risk_high_color: [1.0, 0.5, 0.0]       # orange
    risk_critical_color: [1.0, 0.0, 0.0]   # red
```
