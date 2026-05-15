# Sensor Ablation Experiments — RangerNav-Fusion

## Purpose

Quantify the contribution of each sensor and each processing module to overall navigation performance. These experiments isolate the effect of MID360S, D435i, fusion, prediction, and risk awareness.

---

## Experiment SA-1: Only MID360S

### Configuration
- **Active**: MID360S
- **Disabled**: D435i (camera unplugged or node not launched)
- **Disabled**: sensor_fusion_node, obstacle_predictor_node, risk_evaluator_node
- **Nav2**: DWB differential (baseline config)

### Purpose
Establish single-sensor LiDAR baseline. Quantify performance without near-field sensing.

### Tests
- Static navigation (Exp 1)
- Narrow passage (Exp 2)
- Pedestrian crossing (Exp 3)

### Hypotheses
- MID360S performs well for SLAM and mid-range obstacles (> 0.5m)
- Fails or degrades for obstacles in blind zone (< 0.5m from LiDAR)
- Cannot detect low obstacles (< 0.3m height) that are below the LiDAR scan slice

### Data to Collect
In addition to standard metrics:
- Number of objects missed in D435i FOV that MID360S did not detect
- Number of false positives (MID360S noise at range > 15m)
- Minimum obstacle distance actually detected

---

## Experiment SA-2: Only D435i

### Configuration
- **Disabled**: MID360S
- **Active**: D435i
- **Disabled**: sensor_fusion_node, obstacle_predictor_node
- **Nav2**: DWB differential; `/scan` from D435i pointcloud_to_laserscan only

### Purpose
Quantify D435i-only navigation capability. Show limitations of single-camera perception.

### Tests
- Static navigation (Exp 1 — but with re-localization, since no MID360S SLAM)

**Note**: D435i-only SLAM is not possible with this setup. Use a pre-built map from MID360S and test navigation only.

### Hypotheses
- Excellent near-field obstacle detection (0.2–1.0m)
- Good low obstacle detection (water bottles, cables, small objects)
- Fails at ranges > 4m (narrow FOV limits forward perception)
- Degraded performance in bright sunlight (IR interference)
- Cannot handle obstacles approaching from sides or behind

### Data to Collect
- Detection rate by range: 0.2–1m, 1–2m, 2–3m, 3–4m
- False negative rate (obstacles present but not detected)
- Performance degradation in different lighting conditions

---

## Experiment SA-3: MID360S + D435i Fusion

### Configuration
- **Active**: Both sensors
- **Active**: sensor_fusion_node (association + merging + confidence)
- **Disabled**: obstacle_predictor_node (no prediction)
- **Nav2**: DWB differential

### Purpose
Quantify the benefit of multi-sensor fusion over single sensors.

### Tests
All Experiments 1–4.

### Hypotheses
- Higher obstacle detection rate than either sensor alone
- Lower false negative rate (D435i covers blind zone)
- Near-field safety zone reduces collision risk at close range
- Fusion weight correctly favors MID360S at range and D435i near-field

### Data to Collect
- Detection rate vs. single sensor baselines
- False positive count (does D435i noise create phantom obstacles?)
- False negative count (are there still missed obstacles?)
- Fusion improvement ratio = success_rate(fusion) / max(success_rate(mid360), success_rate(d435i))

---

## Experiment SA-4: Fusion Without Prediction

### Same as SA-3
Prediction is already disabled in SA-3. This is the baseline for comparison with SA-5.

---

## Experiment SA-5: Fusion With Prediction

### Configuration
- **Active**: Both sensors, fusion, tracking, prediction (obstacle_predictor_node)
- **Disabled**: risk_evaluator_node (no TTC risk in cost function)
- **Nav2**: DWB differential

### Purpose
Isolate the effect of obstacle trajectory prediction on navigation performance.

### Tests
Pedestrian crossing (Exp 3).

### Hypotheses
- Prediction provides 0.5–2.0s earlier response to crossing pedestrian
- Larger minimum distance to dynamic obstacle (robot starts avoiding earlier)
- Potential false positive: if prediction is wrong, robot may brake for a false projected path

### Data to Collect
- Minimum obstacle distance (compare SA-3 vs SA-5)
- Time of first avoidance maneuver (when does robot start deviating/slowing)
- Prediction error: deviation between predicted and actual trajectory at 1.0s and 2.0s

---

## Experiment SA-6: Fusion With Risk-Aware Planner (Full System)

### Configuration
- **Active**: Both sensors, fusion, tracking, prediction, risk evaluator, replan manager
- **Nav2**: DWB omni with risk critic enabled

### Purpose
Evaluate the complete RangerNav-Fusion system against all baselines.

### Tests
All Experiments 1–5.

### Hypotheses
- Full system achieves highest success rate and lowest collision rate
- TTC-based risk evaluation enables earlier, smoother avoidance
- Mode-aware chassis uses lateral motion efficiently
- Replan manager handles persistent blockages without getting stuck
- Sensor confidence weighting reduces false alarms

---

## Summary: 6 Ablation Configurations

| Config | MID360S | D435i | Fusion | Track | Predict | Risk | Replan | Nav2 Mode |
|--------|---------|-------|--------|-------|---------|------|--------|-----------|
| SA-1 | ON | OFF | OFF | OFF | OFF | OFF | OFF | Diff |
| SA-2 | OFF | ON | OFF | OFF | OFF | OFF | OFF | Diff |
| SA-3 | ON | ON | ON | ON | OFF | OFF | OFF | Diff |
| SA-4 | ON | ON | ON | ON | OFF | OFF | OFF | Diff |
| SA-5 | ON | ON | ON | ON | ON | OFF | OFF | Diff |
| SA-6 | ON | ON | ON | ON | ON | ON | ON | Omni |

SA-3 and SA-4 are intentionally identical — SA-4 is the baseline for comparing with SA-5 (prediction). If the user prefers, SA-4 can be removed from the paper.

## Expected Results Table Template

| Metric | SA-1 | SA-2 | SA-3 | SA-5 | SA-6 |
|--------|------|------|------|------|------|
| Success rate (%) | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Collision rate (%) | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Min obstacle distance (m) | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| False positive count | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| False negative count | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Emergency stop count | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Navigation time (s) | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Computation time (ms) | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |
| Sensor dropout cases | [TBD] | [TBD] | [TBD] | [TBD] | [TBD] |

---

## Additional: Sensor Degradation Experiments (Optional)

### SA-7: Varying Lighting (D435i)
- Indoor fluorescent, indoor dim, outdoor overcast, outdoor sunny
- Measure D435i detection rate vs lighting condition

### SA-8: Varying LiDAR Point Density
- Point filter: 1 (full density), 2 (half), 4 (quarter)
- Measure obstacle detection rate vs point density
