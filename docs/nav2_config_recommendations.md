# Nav2 Configuration Recommendations for Ranger Mini 2.0

## Current State Analysis

Reference file: `src/ranger_nav/config/nav2_params.yaml`

The current configuration treats Ranger Mini 2.0 as a differential-drive robot. Below are specific recommended changes.

---

## 1. AMCL Motion Model

**Current** (line 32):
```yaml
robot_model_type: nav2_amcl::DifferentialMotionModel
```

**Issue**: The particle filter uses a differential motion model that only propagates `(vx, wz)` and assumes no lateral velocity. When Ranger executes oblique/side-slip commands, AMCL cannot correctly predict the robot's motion, causing localization error accumulation.

**Note**: The current launch files use `slam_toolbox` in localization mode, not AMCL. This AMCL configuration is dormant.

**Recommendation**: If switching to AMCL for comparison experiments:
```yaml
robot_model_type: nav2_amcl::OmniMotionModel
```

The OmniMotionModel supports `vx`, `vy`, and `wz` in the prediction step. `alpha1–alpha5` noise parameters must be re-tuned for the Ranger platform — current values are generic defaults.

**Action**: Keep slam_toolbox localization as default for now. Add AMCL+OmniMotionModel as an alternative config file (`nav2_params_amcl_omni.yaml`) for ablation experiments.

---

## 2. DWB Lateral Velocity

**Current** (lines 92-94, 100-106):
```yaml
min_vel_y: 0.0
max_vel_y: 0.0
vy_samples: 0
acc_lim_y: 0.0
dec_lim_y: 0.0
```

**Issue**: DWB generates zero lateral velocity candidates. Ranger Mini 2.0's oblique/side-slip capability is completely unused.

**Recommendation** (conservative, for real-robot safety):
```yaml
min_vel_y: -0.3
max_vel_y: 0.3
vy_samples: 11          # -0.3 to +0.3 in 11 samples
acc_lim_y: 0.5
dec_lim_y: -0.5
```

**Rationale**: Lateral acceleration is limited to 0.5 m/s² for stability. Max lateral speed of 0.3 m/s is conservative — Ranger Mini 2.0 can physically do more, but lateral obstacle avoidance at higher speeds risks instability on uneven terrain.

**Verification**: Test lateral obstacle avoidance with a single obstacle placed 1m to one side of the path. DWB should generate trajectories that use `vy` to slide away from the obstacle.

---

## 3. Max Angular Velocity

**Current** (line 95):
```yaml
max_vel_theta: 1.5
```

**Issue**: Ranger Mini V2 `max_angular_speed` is 4.8 rad/s (`ranger_params.hpp:56`). But 1.5 rad/s × min_turn_radius (0.476m) = 0.71 m/s — reasonable for navigation. However, the behavior server's spin recovery uses `max_rotational_vel: 1.5` (line 162), which is fine.

**Recommendation**: Reduce for safety:
```yaml
max_vel_theta: 0.8
```
And behavior server:
```yaml
max_rotational_vel: 0.8
```

Controls rotation speed during path following and recovery spins. Real-robot testing should validate if 0.8 rad/s is sufficient for in-place rotation recovery.

---

## 4. Robot Footprint

**Current** (line 260-261): Comment says "set in launch" but no footprint is actually set in any launch file.

**Issue**: Without an explicit footprint, Nav2 defaults to a point robot or minimal radius. This is **unsafe** for a 0.55m × 0.45m robot.

**Recommendation**: Add to both global_costmap and local_costmap:
```yaml
robot_radius: 0.35   # conservative circle covering 0.55×0.45 rect
```
Or explicit polygon:
```yaml
footprint: "[[-0.275, -0.225], [0.275, -0.225], [0.275, 0.225], [-0.275, 0.225]]"
```

**Rationale**: The rectangular footprint is more accurate but may cause planning failures in narrow passages. Start with `robot_radius: 0.35` and switch to polygon for final experiments.

---

## 5. Inflation Radius

**Current** (line 213, 257):
```yaml
inflation_radius: 0.55
cost_scaling_factor: 3.0
```

**Issue**: With `robot_radius: 0.35`, total safety distance = 0.35 + 0.55 = 0.90m. This may be too conservative for narrow corridors (e.g., standard 1.2m doorways).

**Recommendation**:
```yaml
inflation_radius: 0.40    # total: 0.35 + 0.40 = 0.75m
cost_scaling_factor: 5.0  # steeper decay, obstacles matter less at distance
```

**Rationale**: Higher `cost_scaling_factor` (5.0 vs 3.0) means cost drops faster with distance. Obstacles at 0.75m are nearly free, but obstacles at 0.5m are heavily penalized. This allows passing through 1.2m doorways while still avoiding collisions.

---

## 6. Controller Frequency

**Current** (line 66):
```yaml
controller_frequency: 20.0
```

**Issue**: 20 Hz means 50ms between control commands. For a robot moving at 1.0 m/s, this is 5cm per cycle. Reasonable, but the CAN bus update rate is 50 Hz (20ms). At 20 Hz control, the chassis runs open-loop for 2.5 CAN cycles between Nav2 updates.

**Recommendation**: Keep 20 Hz for now. 50 Hz on Jetson may strain compute. Evaluate at:
- 20 Hz: baseline
- 30 Hz: if dynamic obstacle experiments show jerky response
- Do not exceed 50 Hz (CAN bus limit)

---

## 7. MPPI vs DWB

**Current**: DWB only.

**Recommendation**: Add MPPI as an alternative controller for comparison experiments.

MPPI advantages for this project:
1. **Sample-based**: naturally handles the non-holonomic constraint — can encode mode limits in the dynamics model
2. **Stochastic**: more robust to sensor noise than DWB's deterministic scoring
3. **GPU-free**: MPPI runs on CPU; 1000–2000 samples at 20 Hz is feasible on Jetson Orin

**Action**: Create `config/nav2_risk_mppi.yaml` with MPPI controller and add a launch argument `controller:=dwb|mppi`.

---

## 8. Constraint: Nav2 Holonomic Planner vs Mode-Based Chassis

**The fundamental problem**: Nav2 planners assume the robot can execute any `(vx, vy, wz)` within velocity limits. Ranger Mini 2.0 cannot.

### Approach: Planner-Agnostic, Controller-Only

- **Global planner**: Keep NavfnPlanner (or SmacPlanner). It plans a geometric path (ignoring kinematics). This is fine — the path is a reference.
- **Local controller**: DWB/MPPI generates `cmd_vel` candidates. The `ranger_messenger.cpp` mode mapping converts them to feasible chassis commands.
- **Constraint**: Nav2's holonomic assumption is violated, but **this is the core contribution**. We are NOT fixing the planner. We are building a mode-aware controller + risk evaluation layer on top.

### What NOT to do:
- Do NOT modify the global planner to output mode-specific paths (too invasive)
- Do NOT create a custom Nav2 planner plugin (too fragile with upstream changes)

### What TO do:
- Enable `vy_samples` in DWB so it considers lateral trajectories
- Let `ranger_messenger.cpp` handle the vx+vy→oblique mapping (already done)
- The risk critic penalizes trajectories that would cause excessive mode switching
- Document the mismatch as an accepted system limitation

---

## 9. Summary: Three Config Files

| File | Purpose | Key Differences |
|------|---------|----------------|
| `nav2_baseline_dwb.yaml` | Baseline: differential DWB (no vy) | `max_vel_y: 0.0`, `vy_samples: 0` |
| `nav2_omni_dwb.yaml` | Omni DWB with lateral capability | `max_vel_y: 0.3`, `vy_samples: 11` |
| `nav2_risk_mppi.yaml` | MPPI with risk critic (optional) | MPPI controller, risk-aware params |

**Recommendation**: Start with `nav2_omni_dwb.yaml` as the main config. Create the baseline and MPPI configs for ablation experiments.

---

## 10. Additional Tuning Parameters

| Parameter | Current | Recommended | Reason |
|-----------|---------|-------------|--------|
| `sim_time` (DWB) | 2.5s | 2.0s | Slightly shorter horizon for dynamic environments |
| `xy_goal_tolerance` | 0.15m | 0.20m | Relax slightly — Ranger's Ackermann can't do sub-cm positioning |
| `yaw_goal_tolerance` | 0.1 rad | 0.15 rad | Relax for non-holonomic approach |
| `transform_tolerance` | 0.3s | 0.2s | Stricter TF timing |
| `vx_samples` | 20 | 20 | OK |
| `vtheta_samples` | 40 | 30 | Fewer samples, less compute |
| `ObstacleFootprint.scale` | 20.0 | 25.0 | Slightly more conservative obstacle avoidance |
| `controller_frequency` | 20.0 | 20.0 | Keep for now; evaluate 30 Hz later |

---

## 11. Costmap Recommendations

### Global Costmap
- Add `footprint` or `robot_radius`
- Keep `obstacle_layer` with `/scan` only (MID360S)
- Do NOT add D435i to global costmap (avoids noise in static map)

### Local Costmap
- Add `footprint` or `robot_radius`
- Add D435i `/d435i/scan` as additional observation source
- Consider adding `voxel_layer` with both scan sources
- Reduce local costmap `width/height` to 4m (from 6m) for D435i's 4m range

### Inflation
- Global: `inflation_radius: 0.55` (keep, safety for static map errors)
- Local: `inflation_radius: 0.40` (reduce, dynamic avoidance handles near-field)
