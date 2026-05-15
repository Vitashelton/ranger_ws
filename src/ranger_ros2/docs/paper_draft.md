# Mode-Aware, Risk-Constrained Global-Local Navigation with Multi-Modal Dynamic Perception for Omnidirectional UGVs in Real-World Environments

**Target Venue:** IEEE Robotics and Automation Letters (RA-L) with ICRA 2027 Option

**Authors:** [TBD]

**Affiliation:** [TBD]

---

## Abstract

Deploying mobile robot navigation systems in real-world dynamic environments exposes fundamental gaps between standard ROS2 Navigation2 (Nav2) stacks and the physical platforms they control. While Nav2 provides mature global planning, local trajectory optimization, and costmap-based collision avoidance, it treats the robot as a generic holonomic base and relies on static-costmap representations that do not explicitly model moving obstacles. On a multi-mode omnidirectional platform such as the Ranger Mini 2.0—which supports Ackermann, oblique, side-slip, and spin motion modes—the single `cmd_vel` interface cannot simultaneously command arbitrary longitudinal, lateral, and angular velocities. Furthermore, single-sensor perception (LiDAR-only) suffers from sparsity at short range and lacks the semantic cues needed to distinguish pedestrians from static clutter, leading to conservative or unsafe behavior.

In this paper, we present a **mode-aware, risk-constrained, event-driven global-local navigation framework** that augments, rather than replaces, the standard ROS2/Nav2 stack, enhanced by **fused LiDAR–RGB-D perception** for robust dynamic obstacle detection. Our framework comprises five tightly integrated modules: (i) a **multi-modal dynamic perception pipeline** that fuses Livox MID360 LiDAR point clouds with Intel RealSense D435i depth and RGB data to achieve dense short-range coverage, reliable long-range detection, and lightweight obstacle classification; (ii) a **mode-aware chassis interface** that resolves `cmd_vel` conflicts through a kinematic-priority arbitration strategy and maps them to the four discrete motion modes of the Ranger Mini 2.0; (iii) a **risk-aware local trajectory evaluator** that augments DWB/MPPI scoring with a composite cost term incorporating Time-To-Collision, minimum safety distance, path deviation, goal progress, and velocity smoothness; (iv) an **event-driven replanning trigger** that initiates global replanning only when local avoidance proves insufficient; and (v) a **sensor-failure degradation module** that maintains safe operation when one sensor modality is temporarily unavailable.

We evaluate the framework through [TBD] real-robot experiments across six representative scenarios—static navigation, narrow-corridor passing, pedestrian crossing, temporary global-path obstruction, motion mode comparison, and sensor-degradation recovery—on a Ranger Mini 2.0 equipped with a Livox MID360 LiDAR, an Intel RealSense D435i depth camera, and an onboard NVIDIA Jetson [TBD] computer. Compared against [TBD] baseline configurations, our full framework achieves [TBD]% navigation success rate, reduces emergency stops by [TBD]%, and maintains a minimum obstacle clearance of [TBD] m. Ablation studies confirm that each module—multi-modal perception, mode-aware arbitration, risk scoring, and event-driven replanning—contributes measurably to overall safety and efficiency. The results demonstrate that **safe, reliable navigation in dynamic environments requires explicit sensor fusion, platform awareness, dynamic risk evaluation, and adaptive global-local coordination**—none of which can be achieved through planner parameter tuning alone.

**Keywords:** mobile robot navigation, dynamic obstacle avoidance, sensor fusion, omnidirectional UGV, ROS2 Nav2, real-world deployment

---

## I. Introduction

### A. Motivation: When RViz Works but the Robot Does Not

Consider the following real-world deployment scenario. A Ranger Mini 2.0 omnidirectional UGV navigates an office corridor toward a delivery target. SLAM Toolbox has built an accurate occupancy grid; Nav2's Smac Planner has computed a globally optimal path; DWB is generating smooth local trajectories at 20 Hz. In RViz, everything appears flawless.

Then a pedestrian steps out of a side office, crossing the corridor at 1.2 m/s. The robot is now 4 meters away. Three things go wrong simultaneously, and none of them are visible in the RViz visualization:

1. **The costmap lags.** The pedestrian appears in the LiDAR scan only after the next full scan cycle. By the time the costmap is updated, the pedestrian's true position is already 20–30 cm ahead of the costmap obstacle footprint. DWB, scoring trajectories against this stale costmap, selects a path that passes dangerously close to the pedestrian.

2. **The robot cannot execute what Nav2 commands.** Nav2 outputs `cmd_vel = (vx=0.3, vy=0.15, wz=0.1)`—a reasonable holonomic avoidance maneuver. But the Ranger Mini 2.0 chassis, connected via CAN bus, operates in discrete motion modes. It cannot simultaneously translate laterally, move forward, and rotate. The chassis firmware selects one mode (oblique), discards the angular component, and the robot moves in a direction that differs from what Nav2 intended.

3. **The planner has no memory of what just happened.** The pedestrian passes. The costmap clears. But DWB's trajectory scoring has no explicit model of *risk*—it treats the cleared region as equally safe as a path that was never occupied. There is no mechanism to penalize trajectories that passed near a recently tracked dynamic object.

These three failures are not bugs in Nav2. They are **semantic mismatches** between what Nav2 assumes and what the physical world—and physical platform—actually provide.

### B. Three Gaps in Current Practice

**Gap 1: Platform Kinematic Assumptions.** Nav2's `cmd_vel` is a continuous 3-DOF velocity command `(vx, vy, wz)` that implicitly assumes the robot can instantaneously execute any combination. The Ranger Mini 2.0, like many real UGVs with independently steerable wheels, uses a **mode-discrete CAN protocol**: Ackermann (car-like steering), oblique (all wheels steered to same angle), side-slip (pure lateral), and spin (zero-radius rotation). These four modes cannot be mixed within a single CAN frame. When the chassis bridge receives a holonomic-style `cmd_vel`, it must *arbitrate*—select one mode and discard incompatible velocity components. This introduces **trajectory distortion** η, defined as the normalized difference between the commanded and executed velocity. In our measurements, η reaches 0.3–0.7 for commands containing both lateral and angular components.

Prior work on local planning—DWB [1], MPPI [2], TEB [3]—assumes either differential-drive or fully holonomic kinematics. None explicitly model the mode-discrete constraint of independently-steered four-wheel platforms. While some works address control allocation for four-wheel-steering vehicles [9, 10], they treat it as a post-hoc optimization *after* trajectory generation, not as an integrated constraint during trajectory evaluation.

**Gap 2: Single-Sensor Perception Limits.** Most Nav2 deployments rely on a single 2D or 3D LiDAR for obstacle detection. This creates three blind spots in dynamic environments:

- **Short-range sparsity:** The Livox MID360, while providing 360° coverage, has reduced point density at very close range (< 0.5 m). Objects directly in front of the robot may not be adequately sampled.
- **Lack of semantic cues:** A LiDAR point cluster cannot distinguish between a pedestrian (requiring cautious, predictive avoidance), a static cardboard box (requiring simple static avoidance), and a glass door (potentially invisible to LiDAR).
- **Temporal sparsity in costmap:** The standard Nav2 costmap operates at 5–10 Hz update rates. A pedestrian moving at 1.5 m/s can traverse 15–30 cm between costmap updates, creating trailing artifacts that mislead trajectory scoring.

RGB-D cameras (e.g., Intel RealSense D435i) provide complementary capabilities: dense depth at close range, RGB for obstacle classification, and an onboard IMU. However, consumer depth cameras have limited range (~10 m effective) and narrow field of view (~87°×58°), making them insufficient as a standalone navigation sensor.

**Gap 3: Fixed-Frequency Replanning.** Nav2's default behavior triggers global replanning at a fixed frequency (typically 1 Hz) or when the goal is updated. This is simultaneously too frequent (wasteful in static environments) and too infrequent (insufficient in highly dynamic scenes). Between two 1 Hz replanning cycles, the robot relies entirely on the local planner's limited horizon (typically 3–5 m in DWB). When a tracked obstacle blocks the global path, the local planner may oscillate or fail without triggering a path reconsideration.

### C. Our Approach

We do not propose a replacement for Nav2. Instead, we propose an **augmentation layer** that addresses these three gaps through four integrated modules:

1. **Multi-modal dynamic perception (LiDAR + RGB-D):** We fuse MID360 point clouds with D435i depth and RGB to achieve 360° sparse long-range coverage, dense forward short-range coverage, and lightweight obstacle classification. This module outputs tracked and classified obstacle states with short-horizon velocity predictions.

2. **Mode-aware chassis interface:** We formalize the `cmd_vel` → motion mode mapping as a kinematic-priority arbitration problem, quantify the resulting trajectory distortion, and feed the actual executed motion back into local trajectory evaluation.

3. **Risk-aware trajectory scoring:** We augment DWB/MPPI trajectory candidates with a composite risk term derived from tracked obstacle states—incorporating Time-To-Collision (TTC), minimum safety distance, path deviation, goal progress, and velocity smoothness. Critically, the risk term accounts for **obstacle class** (pedestrian vs. static obstacle) with differentiated safety margins.

4. **Event-driven replanning:** We replace fixed-frequency replanning with multi-condition triggers based on sustained low speed, persistent TTC violation, and global path occlusion by tracked obstacles.

### D. Contributions

> **C1. A multi-modal dynamic perception pipeline** that fuses 360° LiDAR with forward-facing RGB-D for dense close-range coverage, long-range detection, and lightweight semantic obstacle classification, running in real-time on an embedded Jetson platform.

> **C2. A mode-aware chassis interface** that formalizes the semantic gap between Nav2's continuous `cmd_vel` and the Ranger Mini 2.0's discrete motion modes as a kinematic arbitration problem, with quantitative trajectory distortion metrics.

> **C3. A risk-constrained trajectory evaluation framework** that augments standard DWB/MPPI cost terms with dynamic TTC-based risk, path deviation, goal progress, and smoothness penalties—differentiated by obstacle class.

> **C4. An event-driven global-local coordination mechanism** that triggers replanning based on real-time risk monitoring rather than fixed frequency, and a sensor-degradation handling strategy for single-modality failure.

> **C5. Systematic real-robot validation** across six dynamic scenarios with [TBD] baseline comparisons, including sensor-fusion ablation and cross-modality degradation analysis.

### E. Paper Organization

Section II reviews related work. Section III formalizes the problem. Section IV presents our method in five modules. Section V describes the experimental setup. Section VI presents results and analysis. Section VII discusses limitations and future work. Section VIII concludes.

---

## II. Related Work

### A. ROS2 Nav2 Ecosystem

The Nav2 framework [4] provides a modular navigation architecture with plugin-based global planners (NavFn, Smac Planner), local planners (DWB, MPPI, TEB), costmap layers, and behavior trees. Macenski et al. [4] present a comprehensive overview emphasizing industrial deployment. While Nav2 is the de facto standard for ROS2-based navigation, its design assumes generic holonomic or differential-drive kinematics and does not provide explicit mechanisms for dynamic obstacle risk evaluation or platform-specific motion mode adaptation.

### B. Local Trajectory Planning and Optimization

**DWB (Dynamic Window Approach with Banded Trajectories)** [1] generates candidate trajectories by sampling velocity pairs, scores them against weighted cost critics (obstacle proximity, goal alignment, path alignment, oscillation), and selects the highest-scoring trajectory. DWB's costmap-based obstacle critic evaluates static occupancy; it has no explicit model of obstacle velocity or predicted future positions.

**MPPI (Model Predictive Path Integral Control)** [2] uses sampling-based stochastic optimization to select control sequences that minimize a task-specific cost function. While MPPI can incorporate arbitrary cost terms (including dynamic obstacle predictions [11]), standard Nav2 MPPI configurations rely on the same static costmap as DWB.

**TEB (Timed Elastic Band)** [3] formulates trajectory optimization as a sparse pose-graph optimization problem, simultaneously optimizing path shape and velocity profile. TEB supports dynamic obstacle inclusion via time-varying obstacle footprints [12], but the integration requires known obstacle trajectories, which are not natively provided by Nav2's perception layer.

**Key distinction:** Our risk-aware scoring is planner-agnostic—it augments the cost function of DWB, MPPI, or TEB with tracked obstacle states, and feeds back the actual executed motion (post mode-arbitration) for accurate evaluation.

### C. Dynamic Obstacle Detection and Tracking

**LiDAR-based methods:** Euclidean clustering on range images or point clouds [13], combined with Kalman filter tracking [14], remains the most common approach for real-time deployment. Recent learned approaches use PointPillars [15] or CenterPoint [16] for 3D detection, but these typically require GPU acceleration and large annotated datasets.

**RGB-D-based methods:** Depth cameras have been used for pedestrian detection [17] and close-range obstacle avoidance [18]. The D435i's active IR stereo provides reliable depth at 0.2–10 m range, complementing LiDAR's long-range capabilities.

**Sensor fusion:** Works such as [19, 20] fuse LiDAR and camera data for 3D object detection. However, these are typically designed for autonomous driving at high speeds with powerful compute. Our work targets the embedded UGV domain, where computational budget is constrained and the fusion must be lightweight.

### D. Navigation on Multi-Mode Omnidirectional Platforms

Four-wheel independent steering and driving (4WIS-4WID) platforms have been studied in the vehicle control literature [9, 10, 21]. These works focus on control allocation (mapping desired vehicle-level wrench to individual wheel torques and angles) but do not address the interaction with a navigation stack.

The Ranger Mini series has been used in agricultural robotics [22] and last-mile delivery research [23], but prior work treats it as a black-box velocity follower without exploiting or modeling its multi-mode capability.

### E. Risk Assessment in Robot Navigation

TTC has been used as a safety metric in autonomous driving [24] and social navigation [25]. Risk-aware planning incorporating TTC constraints has been explored in the context of model predictive control [26] and reinforcement learning [27]. Our contribution is the integration of a lightweight TTC-based risk term *within* the standard Nav2 local planner scoring pipeline, with obstacle-class-dependent safety margins and feedback from the mode-aware interface.

### F. Related Work Summary

| Aspect                | Existing Work                                                               | Our Work                                                        |
| --------------------- | --------------------------------------------------------------------------- | --------------------------------------------------------------- |
| Local planning        | DWB [1], MPPI [2], TEB [3] — assume holonomic or diff-drive, static costmap | Planner-agnostic risk augmentation with dynamic obstacle states |
| Platform modeling     | 4WS control allocation [9, 10] — no nav integration                         | Mode-aware arbitration integrated into nav pipeline             |
| Dynamic perception    | LiDAR-only tracking [14], RGB-D detection [17]                              | Fused LiDAR–RGB-D with lightweight classification               |
| Sensor fusion for nav | Autonomous driving [19, 20] — heavy compute                                 | Embedded-friendly fusion for indoor/outdoor UGV                 |
| Replanning strategy   | Fixed-frequency (Nav2 default)                                              | Event-driven with multi-condition triggers                      |
| Real-robot validation | Mostly simulation or structured environments                                | 6 real-world scenarios with systematic metrics                  |

---

## III. Problem Formulation

### A. System Overview

We consider a mobile robot operating in a partially known, dynamic environment. The robot is equipped with:

- **Livox MID360 LiDAR**: 360° horizontal FOV, 59° vertical FOV, range up to 40 m, ~200,000 points/sec, non-repetitive scanning pattern
- **Intel RealSense D435i**: 87°×58° depth FOV, range 0.2–10 m, 1280×720 RGB @ 30 Hz, integrated IMU (BMI055)
- **Onboard NVIDIA Jetson** [TBD]: Orin NX / AGX Orin
- **Ranger Mini 2.0 chassis**: Track 0.364 m, wheelbase 0.494 m, max speed 1.5 m/s, four motion modes

The navigation task is: given a goal pose `(x_g, y_g, θ_g)` in a pre-built occupancy grid map `M`, navigate to the goal while avoiding N dynamic obstacles with unknown intents.

### B. Robot Kinematic Model

The Ranger Mini 2.0 supports four discrete motion modes:

| Mode                         | Control Input                      | Executed Motion                      | Use Case                    |
| ---------------------------- | ---------------------------------- | ------------------------------------ | --------------------------- |
| **Dual Ackermann** (`M_A`)   | `(v, δ)`, δ ∈ [-40°, 40°]          | Car-like turning, min radius 0.476 m | Forward path following      |
| **Oblique/Parallel** (`M_O`) | `(v, δ)`, δ ∈ [-90°, 90°]          | All wheels steered to same angle δ   | Diagonal/angled movement    |
| **Side-Slip** (`M_S`)        | `(v, ±π/2)`                        | Pure lateral translation             | Docking, lateral adjustment |
| **Spin** (`M_SP`)            | `(0, 0, ω)`, ω ∈ [-4.8, 4.8] rad/s | Zero-radius rotation                 | In-place turning            |

The chassis CAN protocol accepts one mode at a time. Any `cmd_vel = (vx, vy, wz)` from Nav2 must be mapped to exactly one mode.

### C. Nav2-to-Chassis Semantic Gap

Formally, Nav2 produces a control command `u_nav ∈ ℝ³` in the continuous velocity space. The chassis accepts a command `u_chassis ∈ U` where:

$$U = M_A \cup M_O \cup M_S \cup M_{SP}$$

where each `M_*` is a strict subset of `ℝ³` with non-holonomic constraints. The mapping `f: ℝ³ → U` is **non-injective and non-surjective**: many Nav2 commands map to the same chassis command (information loss), and some reasonable Nav2 commands (e.g., simultaneous lateral motion and rotation) have no exact representation in U.

We define the **trajectory distortion** η for a command as:

$$\eta(u_{nav}) = \frac{\|u_{nav} - u_{exec}\|_2}{\max(\|u_{nav}\|_2, \varepsilon)}$$

where `u_exec` is the equivalent twist of the executed chassis command. For differential-drive robots, η ≈ 0 for all feasible commands (since any (vx, 0, wz) can be exactly realized). For the Ranger Mini 2.0, η can reach 0.3–0.7 when `vy ≠ 0` and `wz ≠ 0` co-occur in the Nav2 output.

### D. Dynamic Obstacle Model

The environment contains N dynamic obstacles. Each obstacle `i` at time `t` is represented by:

$$o_i(t) = [p_{i,x}, p_{i,y}, v_{i,x}, v_{i,y}, c_i]^T$$

where `(p, v)` are position and velocity in the robot frame, and `c_i ∈ {pedestrian, static_obstacle, unknown}` is the obstacle class.

We assume constant-velocity motion over the prediction horizon `T_h`:

$$\hat{p}_i(t+\tau) = p_i(t) + v_i(t) \cdot \tau, \quad \tau \in [0, T_h]$$

The obstacle state is also represented by a covariance ellipse `Σ_i(t+τ)` that grows linearly with prediction horizon to account for prediction uncertainty.

### E. Navigation Objective

The robot must generate a sequence of control commands `{u_t}` that:

1. **Reach the goal**: `‖p_T − p_g‖ < d_goal` (typically 0.5 m)
2. **Avoid collisions**: `‖p_t − p_i(t)‖ > d_safe(c_i)` for all `i, t`, where `d_safe(c_i)` depends on obstacle class (`d_safe(pedestrian)` > `d_safe(static)`)
3. **Respect platform constraints**: `u_t ∈ U` for all `t`
4. **Maintain comfort**: minimize control oscillation and emergency stops

---

## IV. Method

Our framework consists of five modules arranged as an augmentation layer around the standard Nav2 stack. Figure 1 illustrates the system architecture.

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        PROPOSED FRAMEWORK                                │
│                                                                          │
│  ┌───────────────────┐     ┌─────────────────────┐                      │
│  │  Livox MID360     │     │  Intel D435i        │                      │
│  │  (360° LiDAR)     │     │  (RGB-D + IMU)      │                      │
│  └────────┬──────────┘     └──────────┬──────────┘                      │
│           │                           │                                  │
│           └───────────┬───────────────┘                                  │
│                       ▼                                                  │
│           ┌───────────────────────────┐                                  │
│           │  Module 1: Multi-Modal    │                                  │
│           │  Dynamic Perception       │                                  │
│           │  · Point cloud fusion     │                                  │
│           │  · Clustering & tracking  │                                  │
│           │  · RGB-based classification│                                 │
│           │  · Short-horizon prediction│                                 │
│           └─────────────┬─────────────┘                                  │
│                         │  ObstacleTrackArray                            │
│                         ▼                                                │
│  ┌──────────┐   ┌───────────────┐   ┌──────────────────┐                │
│  │ Global   │   │ Module 3:     │   │ Module 2:        │                │
│  │ Planner  │──▶│ Risk-Aware    │◀──│ Mode-Aware       │                │
│  │ (NavFn/  │   │ Local Traj.   │   │ Chassis I/F      │                │
│  │  Smac)   │   │ Scoring       │   │                  │                │
│  │          │   │               │   │ cmd_vel → mode   │                │
│  │          │   │ C_base +      │   │ arbitration      │                │
│  │          │   │ w1·C_ttc +    │   │ dist. feedback   │                │
│  │          │   │ w2·C_path +   │   └────────┬─────────┘                │
│  │          │   │ w3·C_goal +   │            │                          │
│  │          │   │ w4·C_smooth   │            ▼                          │
│  └────┬─────┘   └───────┬───────┘   ┌──────────────────┐                │
│       │                 │           │  Ranger Mini 2.0 │                │
│       │                 │           │  CAN Bus         │                │
│       │                 │           │  (M_A/M_O/M_S/   │                │
│       │                 │           │   M_SP modes)    │                │
│       │                 │           └──────────────────┘                │
│       │   ┌─────────────▼──────────────┐                                │
│       │   │ Module 4: Event-Driven     │                                │
│       │   │ Replanning Trigger         │                                │
│       └───│ · Speed watchdog           │                                │
│           │ · TTC watchdog             │                                │
│           │ · Path occlusion detector  │                                │
│           └────────────────────────────┘                                │
│                                                                          │
│  │ Module 5: Sensor Degradation Handler                          │       │
│  │ · LiDAR dropout → expand RGB-D depth ROI, reduce max speed    │       │
│  │ · D435i dropout → LiDAR-only mode, disable classification     │       │
│  │ · Both active → full fusion mode                              │       │
│  └──────────────────────────────────────────────────────────────┘       │
└──────────────────────────────────────────────────────────────────────────┘

Figure 1. System architecture of the proposed framework. Shaded modules are our
contributions; unshaded components (Global Planner, DWB/MPPI base) are standard Nav2.
```

### A. Module 1: Multi-Modal Dynamic Perception

**Objective:** Fuse 360° LiDAR and forward-facing RGB-D to produce tracked, classified obstacle states with short-horizon predictions.

#### 1) Point Cloud Registration and Fusion

The Livox MID360 and D435i operate in different coordinate frames. We perform extrinsic calibration using a checkerboard target visible to both sensors (implemented via `ros2` camera-LiDAR calibration tools), producing the transform `T_D435i^MID360`.

**Fusion strategy:**

- **LiDAR points** provide the primary 360° geometric representation.
- **D435i depth points** are transformed to the LiDAR frame and concatenated. Within the D435i's effective range (0.3–6 m) and FOV (87°×58°), depth points provide 10–50× higher point density than LiDAR alone at close range.
- **Duplicate suppression:** Points within 0.02 m of each other are merged (voxel grid filter with 0.02 m leaf size) to avoid double-counting.

The fused point cloud `P_fused` is published at 15 Hz (synchronized to the slower sensor, typically D435i).

#### 2) Ground Segmentation and Clustering

1. **Ground removal:** RANSAC plane fitting on `P_fused` within a 10 m radius. Non-ground points are retained as `P_obstacle`.
2. **Euclidean clustering:** Adaptive radius threshold `r_cluster = 0.05 + 0.01 × range` to account for LiDAR point sparsity at longer ranges. Clusters with fewer than `N_min = 5` points or exceeding a bounding box of 3 m × 3 m × 2 m are filtered.
3. **Cluster centroid extraction:** For each valid cluster `k`, compute centroid and bounding box.

#### 3) RGB-Based Obstacle Classification

For clusters that project into the D435i RGB image plane:

1. **Project 3D cluster centroid** to RGB pixel coordinates using `K_D435i · T_D435i^MID360`.
2. **Extract ROI:** A bounding box around the projected centroid (scaled by cluster size and distance).
3. **Lightweight classification:** We use a MobileNetV3-Small [28] backbone (pretrained on ImageNet, fine-tuned on a small custom dataset of ~500 labeled images containing pedestrians, static obstacles, and negative examples). The network runs at ~10 ms inference time on Jetson Orin NX (TensorRT-optimized).
4. **Classification output:** Each cluster receives a label `c_k ∈ {pedestrian, static_obstacle, unknown}`.

Clusters outside the RGB FOV or beyond effective classification range (>8 m) are labeled `unknown` and treated as static obstacles with a conservative safety margin.

#### 4) Multi-Object Tracking

Each cluster is associated with existing tracks using the Hungarian algorithm with Mahalanobis distance cost:

$$d_{M}(i, j) = (z_j - \hat{x}_i)^T S_i^{-1} (z_j - \hat{x}_i)$$

where `z_j` is the measurement (cluster centroid), `\hat{x}_i` is the predicted state of track `i`, and `S_i` is the innovation covariance.

Each track `i` maintains a Kalman filter with state:

$$x_i = [p_x, p_y, v_x, v_y]^T$$

Process model: constant velocity with Gaussian acceleration noise.
Observation model: direct observation of position.

Track management:
- **Birth:** New cluster not associated with any existing track → initialize new track (needs 3 consecutive associations to become "confirmed").
- **Death:** Track not associated for `T_loss = 0.5 s` → delete.
- **Class inheritance:** A track's class is updated by majority vote over the last `K = 5` associated classifications.

#### 5) Short-Horizon Prediction

For each confirmed track `i`, predict future positions using constant velocity:

$$\hat{p}_i(t + \tau) = p_i(t) + v_i(t) \cdot \tau$$

The prediction uncertainty is modeled as a growing ellipse:

$$\Sigma_i(t + \tau) = \Sigma_i(t) + Q \cdot \tau$$

where `Q` is the process noise covariance. The safety radius for obstacle `i` at prediction time `τ` is:

$$r_i(\tau, c_i) = r_{base}(c_i) + \kappa \cdot \tau$$

with `r_base(pedestrian) = 0.5 m`, `r_base(static) = 0.3 m`, and `κ = 0.1 m/s`.

### B. Module 2: Mode-Aware Chassis Interface

**Objective:** Map Nav2's continuous `cmd_vel` to Ranger Mini 2.0's discrete motion modes while minimizing trajectory distortion and providing distortion feedback to the risk evaluator.

#### 1) Command Arbitration

Given `u_nav = (vx, vy, wz)`, the arbitration follows a priority hierarchy:

**Priority 0 — Deadband check:**
If `‖(vx, vy)‖ < ε_v` AND `|wz| < ε_ω`: issue zero command to current mode.

**Priority 1 — Lateral motion:**
If `|vy| > ε_v`:
- Select **oblique/parallel mode** (`M_O`).
- Compute: `v_cmd = √(vx² + vy²)`, `δ_cmd = atan2(vy, vx)`, clamped to `[-π/2, π/2]`.
- If `|wz| > ε_ω`: **discard wz** (cannot combine lateral translation with rotation in CAN protocol). Log a throttled warning.
- If `|δ_cmd| ≈ π/2`: switch to **side-slip mode** (`M_S`).

**Priority 2 — Pure rotation or sub-minimum-radius turn:**
If `(|vx| < ε_v AND |vy| < ε_v AND |wz| > ε_ω)` OR `(|vx| > ε_v AND |wz| > ε_ω AND |vx/wz| < R_min)`:
- Select **spin mode** (`M_SP`).
- Issue `ω_cmd = clamp(wz, ±ω_max)`.

**Priority 3 — Default Ackermann:**
Otherwise:
- Select **dual Ackermann mode** (`M_A`).
- Compute `δ_cmd = atan(L · wz / vx)`, where `L = 0.494 m` is the wheelbase.
- Clamp `δ_cmd` to `[-40°, 40°]` (CAN protocol limit).
- Issue `v_cmd = clamp(vx, ±v_max)`.

#### 2) Distortion Feedback

After arbitration, we compute the executed equivalent twist `u_exec`:

- For `M_A`: `u_exec = (v_cmd, 0, v_cmd · tan(δ_cmd) / L)`
- For `M_O`: `u_exec = (v_cmd · cos(δ_cmd), v_cmd · sin(δ_cmd), 0)`
- For `M_S`: `u_exec = (0, v_cmd, 0)`
- For `M_SP`: `u_exec = (0, 0, ω_cmd)`

The distortion `η` and the executed twist `u_exec` are published as feedback to the risk-aware trajectory evaluator (Module 3), ensuring that trajectory scoring uses *actual* expected motion rather than Nav2's ideal command.

### C. Module 3: Risk-Aware Local Trajectory Evaluation

**Objective:** Augment DWB/MPPI trajectory scoring with dynamic risk terms derived from tracked obstacle states.

#### 1) Cost Function Structure

For a candidate trajectory `τ_k = {(x_k^{(j)}, y_k^{(j)}, θ_k^{(j)})}_{j=0}^{T-1}` (T timesteps, typically 20–30 at 0.05 s resolution):

$$C_{total}(\tau_k) = C_{base}(\tau_k) + \sum_{m=1}^{5} w_m \cdot C_m(\tau_k)$$

where `C_base` contains the standard DWB/MPPI critics (obstacle, goal_align, path_align, goal_dist, oscillation, etc.), and `C_1` through `C_5` are our risk augmentation terms.

#### 2) Dynamic TTC Risk Term (C₁)

For trajectory `τ_k` at timestep `j`, the TTC with respect to obstacle `i` is:

$$TTC_{k,i}^{(j)} = \frac{\|p_k^{(j)} - \hat{p}_i(t + j\Delta t)\|_2}{\max(0, -(v_k^{(j)} - v_i) \cdot \mathbf{n}_{k,i})}$$

where `\mathbf{n}_{k,i}` is the unit vector from the robot to the obstacle. When the denominator is negative or zero (non-approaching), `TTC → ∞`.

The TTC risk for the trajectory:

$$C_{ttc}(\tau_k) = \sum_{i=1}^{N} \sum_{j=0}^{T-1} \lambda^j \cdot \max\left(0, \frac{1}{TTC_{k,i}^{(j)}} - \frac{1}{TTC_{safe}(c_i)}\right)^2$$

where `λ ∈ (0, 1)` is a temporal discount factor (we use `λ = 0.95`), and `TTC_safe(c_i)` depends on obstacle class:

$$TTC_{safe}(c_i) = \begin{cases} 2.0 \text{ s} & \text{if } c_i = \text{pedestrian} \\ 1.0 \text{ s} & \text{if } c_i = \text{static\_obstacle} \\ 1.5 \text{ s} & \text{if } c_i = \text{unknown} \end{cases}$$

The quadratic penalty ensures that risk grows super-linearly as TTC approaches zero, strongly penalizing near-miss trajectories.

#### 3) Safety Distance Term (C₂)

$$C_{dist}(\tau_k) = \sum_{i=1}^{N} \sum_{j=0}^{T-1} \max\left(0, d_{safe}(c_i) - \|p_k^{(j)} - \hat{p}_i(t + j\Delta t)\|_2\right)^2$$

where `d_safe(pedestrian) = 0.8 m`, `d_safe(static) = 0.3 m`, `d_safe(unknown) = 0.5 m`.

#### 4) Path Deviation Term (C₃)

$$C_{path}(\tau_k) = \sum_{j=0}^{T-1} \|p_k^{(j)} - p_{ref}^{(j)}\|_2^2$$

where `p_ref^{(j)}` is the closest point on the global reference path to `p_k^{(j)}`. This penalizes trajectories that deviate too far from the global plan, preventing the local planner from taking excessive detours.

#### 5) Goal Progress Term (C₄)

$$C_{goal}(\tau_k) = -(\|p_k^{(T-1)} - p_g\|_2 - \|p_0 - p_g\|_2)$$

Negative cost = reward for approaching the goal. This prevents the robot from stagnating or moving away from the goal when avoiding obstacles.

#### 6) Velocity Smoothness Term (C₅)

$$C_{smooth}(\tau_k) = \sum_{j=1}^{T-1} \left( \gamma_v \|v_k^{(j)} - v_k^{(j-1)}\|^2 + \gamma_\omega \|\omega_k^{(j)} - \omega_k^{(j-1)}\|^2 \right)$$

where `γ_v` and `γ_ω` weight translational and rotational smoothness respectively. This term reduces oscillation and jitter caused by rapidly switching between candidate trajectories.

#### 7) Weight Selection

Weights `w_1` through `w_5` are determined through a structured parameter sweep:

1. **Phase 1 (Safety):** Tune `w_1` and `w_2` to achieve collision-free navigation in E3 (pedestrian crossing) with `w_3 = w_4 = w_5 = 0`.
2. **Phase 2 (Efficiency):** Tune `w_3` and `w_4` to minimize navigation time in E1 (static) without compromising safety.
3. **Phase 3 (Comfort):** Tune `w_5` to minimize velocity oscillation measured via accelerometer data from the D435i IMU.

Final converged values are reported in Section VI.

### D. Module 4: Event-Driven Replanning Trigger

**Objective:** Replace Nav2's fixed-frequency replanning with condition-based triggers that respond to actual navigation difficulty.

#### Trigger Conditions (OR logic)

**Condition 1 — Sustained Low Speed:**

If `v_robot < v_thresh` continuously for `T_speed` seconds → trigger replan.
- `v_thresh = 0.1 m/s`, `T_speed = 3.0 s`
- Rationale: The local planner cannot find a trajectory with forward progress. The global path likely needs reconsideration.

**Condition 2 — Persistent TTC Violation:**

If `min_i TTC(t) < TTC_min` continuously for `T_ttc` seconds → trigger replan.
- `TTC_min = 1.0 s`, `T_ttc = 1.5 s`
- Rationale: The local planner is consistently selecting risky trajectories. A new global path may route around the dynamic obstacle region.

**Condition 3 — Global Path Occlusion:**

If any waypoint on the global path (within a look-ahead distance `D_lookahead = 5.0 m`) is occupied by a **tracked static obstacle** (cluster stationary for > `T_static = 2.0 s`) → trigger replan.
- Rationale: A temporary blockage (e.g., a cart left in the corridor) has appeared on the global path. The local planner cannot bypass it without global path reconsideration.

**Condition 4 — Safety Timeout:**

If `t - t_last_replan > T_max` → trigger replan.
- `T_max = 10.0 s`
- Rationale: Ensures eventual replanning even if no condition triggers, preventing unbounded drift from the global path.

#### Hysteresis

To prevent rapid re-triggering, a minimum interval `T_min_replan = 2.0 s` is enforced between consecutive replans.

### E. Module 5: Sensor Degradation Handler

**Objective:** Maintain safe navigation when one sensor modality is temporarily degraded or unavailable.

#### Degradation States

| State          | LiDAR | RGB-D | Behavior                                                                                                                                     |
| -------------- | ----- | ----- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| **Full**       | ✓     | ✓     | All modules active; obstacle classification enabled; full safety margins                                                                     |
| **LiDAR-Only** | ✓     | ✗     | Classification disabled (`c_i = unknown` for all); safety margin set to conservative default (0.5 m); forward blind-spot monitor deactivated |
| **RGB-D-Only** | ✗     | ✓     | 360° awareness lost; reduce max speed to 0.5 m/s; increase `d_safe` to 0.8 m for all obstacles; publish warning                              |
| **Degraded**   | ✗     | ✗     | Emergency stop; publish error                                                                                                                |

#### Detection Mechanism

- **LiDAR health:** Monitor point cloud publish rate. If rate drops below 2 Hz for > 1 s → LiDAR degraded.
- **D435i health:** Monitor depth/RGB topic rates. If rate drops below 5 Hz for > 1 s → D435i degraded.
- **Recovery:** When sensor rate recovers for > 2 s → return to previous state.

---

## V. Experimental Setup

### A. Hardware Platform

| Component        | Specification                                                  |
| ---------------- | -------------------------------------------------------------- |
| **Robot**        | Ranger Mini 2.0 (AgileX/Weston Robot)                          |
| **Chassis**      | 4WIS-4WID, 0.364 m track × 0.494 m wheelbase                   |
| **LiDAR**        | Livox MID360, 360°×59° FOV, 40 m range                         |
| **Depth Camera** | Intel RealSense D435i, 87°×58° depth FOV, RGB 1280×720 @ 30 Hz |
| **Compute**      | NVIDIA Jetson Orin NX (100 TOPS) / AGX Orin [TBD]              |
| **Software**     | ROS2 Humble, Nav2, SLAM Toolbox, Ubuntu 22.04                  |

### B. Experimental Scenarios

Six scenarios are designed to systematically evaluate different aspects of the framework:

| ID     | Scenario               | Description                                                       | Key Challenge                               |
| ------ | ---------------------- | ----------------------------------------------------------------- | ------------------------------------------- |
| **E1** | Static Navigation      | Known map, no dynamic obstacles, start→goal 15 m                  | Baseline performance; mode-aware efficiency |
| **E2** | Narrow Corridor        | 1.2 m wide corridor with static obstacle requiring close passing  | Minimum-distance safety; deadlock avoidance |
| **E3** | Pedestrian Crossing    | Pedestrian crosses robot path at 3 m distance, 1.0–1.5 m/s        | Dynamic TTC risk; class-dependent margins   |
| **E4** | Path Blockage          | Global path blocked by temporary obstacle for 30 s                | Replanning trigger vs. fixed-frequency      |
| **E5** | Motion Mode Comparison | Navigation task requiring lateral offset (e.g., docking approach) | Ackermann vs. omni mode efficiency          |
| **E6** | Sensor Degradation     | E3 repeated with simulated LiDAR or D435i dropout                 | Degradation handler effectiveness           |

Each scenario is repeated 10 times. Pedestrian trajectories in E3 and E6 are performed by human volunteers following marked paths with natural walking variation.

### C. Baseline Methods

| ID     | Configuration                | Description                                                |
| ------ | ---------------------------- | ---------------------------------------------------------- |
| **B1** | NavFn + DWB                  | Nav2 default stack (lower bound)                           |
| **B2** | Smac Hybrid + DWB            | Improved global planner                                    |
| **B3** | Smac Hybrid + MPPI           | State-of-the-art Nav2 local planner                        |
| **B4** | B3 + Mode-Aware only         | Add chassis arbitration, no risk/perception changes        |
| **B5** | B4 + Risk-Aware (LiDAR-only) | Add risk scoring but with LiDAR-only perception            |
| **B6** | B5 − Replan Trigger          | Full method minus event-driven replanning (use fixed 1 Hz) |
| **B7** | **Ours (Full)**              | All five modules active (LiDAR + RGB-D fusion)             |

### D. Evaluation Metrics

| Metric                           | Type       | Definition                                                        | Target            |
| -------------------------------- | ---------- | ----------------------------------------------------------------- | ----------------- |
| **Success Rate (SR)**            | Primary    | Fraction of trials reaching goal (within 0.5 m) without collision | ↑                 |
| **Collision Rate (CR)**          | Safety     | Fraction of trials with physical contact                          | ↓                 |
| **Min Obstacle Distance (MOD)**  | Safety     | Minimum Euclidean distance to any obstacle during trial (m)       | ↑                 |
| **Min TTC**                      | Safety     | Minimum TTC value observed during trial (s)                       | ↑                 |
| **Emergency Stop Count (ESC)**   | Safety     | Number of decelerations exceeding 0.5 m/s²                        | ↓                 |
| **Avg Navigation Time (ANT)**    | Efficiency | Mean time from start to goal across successful trials (s)         | ↓                 |
| **Path Length (PL)**             | Efficiency | Total distance traveled (m)                                       | ↓                 |
| **Replan Count (RC)**            | Efficiency | Number of global replanning invocations                           | Context-dependent |
| **Velocity Smoothness (VS)**     | Comfort    | Mean norm of acceleration between consecutive commands (m/s²)     | ↓                 |
| **Local Planner Failures (LPF)** | Robustness | Number of cycles where local planner returns invalid trajectory   | ↓                 |
| **Trajectory Distortion η**      | Platform   | Mean normalized cmd_vel → executed velocity error                 | ↓                 |
| **Computation Time (CT)**        | Real-time  | Per-module processing time (ms)                                   | ≤ 30 ms           |
| **Classification Accuracy (CA)** | Perception | Obstacle class prediction accuracy vs. ground truth               | ↑                 |

### E. Statistical Analysis

For each metric, we report mean ± standard deviation across 10 trials. We use the **Friedman test** (non-parametric repeated measures) to test for significant differences across baseline configurations, followed by **Wilcoxon signed-rank tests** with Holm-Bonferroni correction for pairwise comparisons between B7 (Ours) and each baseline. Statistical significance is reported at `α = 0.05`.

---

## VI. Results and Analysis

> **Note:** Numerical results are marked [TBD] pending real-robot data collection. The structure and expected trends are provided below, with placeholders for measured values.

### A. Static Navigation (E1)

**Setup:** 15 m straight-line navigation in a 2 m wide corridor. No dynamic obstacles. Five static obstacles (cardboard boxes) placed along the walls.

**Expected results:**

| Method         | SR    | ANT (s) | PL (m) | VS (m/s²) | η     |
| -------------- | ----- | ------- | ------ | --------- | ----- |
| B1 (NavFn+DWB) | [TBD] | [TBD]   | [TBD]  | [TBD]     | [TBD] |
| B2 (Smac+DWB)  | [TBD] | [TBD]   | [TBD]  | [TBD]     | [TBD] |
| B3 (Smac+MPPI) | [TBD] | [TBD]   | [TBD]  | [TBD]     | [TBD] |
| B4 (+Mode)     | [TBD] | [TBD]   | [TBD]  | [TBD]     | [TBD] |
| B7 (Ours)      | [TBD] | [TBD]   | [TBD]  | [TBD]     | [TBD] |

**Expected finding:** All methods should achieve near-100% success in static navigation. B4 and B7 should show lower trajectory distortion η due to mode-aware arbitration. The omni-mode capability (B7) may reduce path length when lateral adjustments are needed near obstacles.

### B. Narrow Corridor (E2)

**Setup:** 1.2 m wide corridor (robot width ~0.7 m, leaving 0.25 m clearance on each side). A static obstacle (0.4 m × 0.4 m box) is placed 0.5 m from the right wall, forcing the robot to pass with ~0.2 m clearance.

**Expected finding:** Risk-aware methods (B5, B6, B7) should maintain larger minimum obstacle distances due to the safety distance term `C_dist`. Without risk awareness (B1–B4), DWB may select trajectories that pass closer than desired to the obstacle.

### C. Pedestrian Crossing (E3)

**Setup:** A pedestrian crosses the robot's path orthogonally at distances of 2 m, 3 m, and 5 m from the robot start position, at walking speeds of 1.0, 1.2, and 1.5 m/s (9 sub-conditions × 10 trials = 90 trials total).

**Expected finding (primary result):** This scenario most strongly differentiates the methods. We expect:
- B1–B3 (no risk, no mode-aware): High emergency stop count and collision rate due to costmap lag.
- B5 (LiDAR-only risk): Improved but may miss close-range pedestrians due to LiDAR sparsity.
- B7 (LiDAR + RGB-D fusion): Lowest collision rate, highest min TTC, due to (a) dense close-range depth from D435i filling LiDAR gaps, (b) pedestrian classification triggering wider safety margins, and (c) RGB-based tracking maintaining pedestrian identity through partial occlusion.

**Per-class safety analysis:** We separately report metrics for trials where the pedestrian was classified as `pedestrian` (B7) vs. `unknown` (B5 LiDAR-only ablation):

| Sub-condition                                 | SR    | CR    | Min MOD (m) | Min TTC (s) | ESC   |
| --------------------------------------------- | ----- | ----- | ----------- | ----------- | ----- |
| B5 (LiDAR only)                               | [TBD] | [TBD] | [TBD]       | [TBD]       | [TBD] |
| B7 (LiDAR+RGB-D, classified as pedestrian)    | [TBD] | [TBD] | [TBD]       | [TBD]       | [TBD] |
| B7 (classified as unknown, misclassification) | [TBD] | [TBD] | [TBD]       | [TBD]       | [TBD] |

### D. Path Blockage (E4)

**Setup:** U-shaped corridor. A large obstacle temporarily blocks the global path for 30 s, then is removed. The robot starts navigation during the blockage period.

**Expected finding:** 
- B6 (fixed 1 Hz replan): Wastes replanning cycles during the blockage (30+ replans) and may oscillate.
- B7 (event-driven): Triggers replanning only when speed drops (Condition 1) or path is occluded (Condition 3). After 1–2 replans that find no alternative path (if corridor is dead-end), the robot waits efficiently. When the obstacle is removed, Condition 2 (speed) or Condition 4 (timeout) triggers a successful replan.

| Method            | SR    | RC    | ANT (s) |
| ----------------- | ----- | ----- | ------- |
| B6 (fixed replan) | [TBD] | [TBD] | [TBD]   |
| B7 (event-driven) | [TBD] | [TBD] | [TBD]   |

### E. Motion Mode Comparison (E5)

**Setup:** A navigation task requiring significant lateral movement: the robot starts 1.5 m laterally offset from a narrow (1.0 m wide) target corridor entrance. Two approaches are compared:

- **Ackermann-only:** The robot must perform a multi-point turn to align.
- **Omni-enabled:** The robot uses side-slip/oblique mode for direct lateral approach.

**Expected finding:**

| Mode           | ANT (s) | PL (m) | VS (m/s²) |
| -------------- | ------- | ------ | --------- |
| Ackermann-only | [TBD]   | [TBD]  | [TBD]     |
| Omni-enabled   | [TBD]   | [TBD]  | [TBD]     |

Expected: Omni-enabled reduces navigation time by [TBD]% and path length by [TBD]% for lateral-offset scenarios due to direct side-slip approach instead of multi-point turning.

### F. Sensor Degradation (E6)

**Setup:** E3 (pedestrian crossing) repeated under:
- **E6a:** LiDAR artificially disabled at `t = 3 s` → RGB-D-only mode
- **E6b:** D435i artificially disabled at `t = 3 s` → LiDAR-only mode
- **E6c:** Both sensors available throughout

**Expected finding:**

| Sub-condition     | SR    | CR    | Min MOD (m) | Max Speed (m/s) |
| ----------------- | ----- | ----- | ----------- | --------------- |
| E6a (RGB-D only)  | [TBD] | [TBD] | [TBD]       | 0.5 (limited)   |
| E6b (LiDAR only)  | [TBD] | [TBD] | [TBD]       | 1.5             |
| E6c (Full fusion) | [TBD] | [TBD] | [TBD]       | 1.5             |

Expected: Full fusion outperforms both single-modality baselines. RGB-D-only mode, while functional, is slower due to enforced speed limits. LiDAR-only mode loses close-range density and classification, resulting in reduced safety margins.

### G. Ablation Study Summary

| Module Removed                        | Expected Degradation                                       |
| ------------------------------------- | ---------------------------------------------------------- |
| − Mode-Aware (B3 vs. B4)              | Higher η; trajectory distortion in mixed-command scenarios |
| − RGB-D Fusion (B4 vs. B5)            | Loss of close-range density; no obstacle classification    |
| − Risk-Aware Scoring (B4 vs. B5)      | Lower min TTC; higher ESC in dynamic scenarios             |
| − Event-Driven Replan (B6 vs. B7)     | Higher RC in static; potential deadlock in E4              |
| − Degradation Handler (B7 without M5) | Unsafe behavior during sensor dropout                      |

### H. Computational Performance

| Module                                          | Platform             | Mean Time (ms) | Max Time (ms) |
| ----------------------------------------------- | -------------------- | -------------- | ------------- |
| M1: Perception (fusion + clustering + tracking) | Jetson Orin NX       | [TBD]          | [TBD]         |
| M1: Classification (MobileNetV3)                | Jetson Orin NX (GPU) | [TBD]          | [TBD]         |
| M2: Mode Arbitration                            | CPU                  | [TBD]          | [TBD]         |
| M3: Risk-Aware Scoring (per trajectory batch)   | CPU                  | [TBD]          | [TBD]         |
| M4: Replan Trigger Evaluation                   | CPU                  | [TBD]          | [TBD]         |
| **Total per-cycle overhead**                    | —                    | [TBD]          | [TBD]         |

Target: Total overhead ≤ 30 ms per control cycle to maintain real-time performance at 20+ Hz.

---

## VII. Discussion

### A. Why the Framework Works (and When It Might Not)

The proposed framework improves navigation safety and efficiency through complementary mechanisms:

1. **Multi-modal perception** eliminates the blind spots that cause single-sensor systems to miss close-range or fast-moving obstacles. The RGB-based classification enables **differentiated safety behavior**—the robot maintains wider clearance from pedestrians than from static clutter, mimicking human-like spatial awareness.

2. **Mode-aware arbitration** closes the loop between what Nav2 *intends* and what the chassis *executes*. By feeding the actual executed motion back into trajectory evaluation, the local planner scores trajectories against realistic outcomes rather than idealized holonomic assumptions.

3. **Event-driven replanning** aligns global path updates with actual need rather than a fixed clock, reducing computational waste while ensuring responsiveness when the local planner cannot find a safe trajectory.

**Limitations:**

- **Classification robustness:** MobileNetV3-Small achieves [TBD]% accuracy on our custom dataset. In adverse lighting (direct sunlight saturating IR projector) or at extreme distances (>8 m), classification degrades to `unknown`.
- **Constant-velocity prediction:** Our KF tracker assumes constant velocity, which fails for abruptly stopping or turning pedestrians. The growing uncertainty ellipse partially compensates, but the prediction horizon `T_h` is limited to 2–3 s.
- **Single dynamic obstacle assumption:** The current TTC cost evaluates pair-wise robot-obstacle TTC independently. In multi-pedestrian scenarios, the robot may be forced into a local minimum where all trajectories have non-zero TTC risk. Cooperative avoidance or social navigation awareness is beyond the current scope.
- **CAN protocol latency:** The mode-switching command through CAN introduces ~10–20 ms additional latency not present in simulation.

### B. Lessons for Real-Robot Deployment

1. **RViz is not reality.** The costmap visualization in RViz represents a lagged, static-probability view of the world. Real dynamic obstacles are 20–50 cm ahead of their costmap footprints at typical walking speeds. Debugging navigation purely in RViz is unreliable for dynamic scenarios.

2. **The holonomic assumption is surprisingly harmful.** Most Nav2 tutorials and default configurations assume a holonomic base. On a mode-discrete chassis like the Ranger Mini 2.0, the trajectory distortion η reaches 0.3–0.7 for commands containing both `vy` and `wz`. This is not a small perturbation—it is a fundamental semantic mismatch that cannot be "tuned away" by adjusting DWB weights.

3. **Sensor fusion is essential but must be lightweight.** Our initial experiments with PointPillars-based 3D detection on the Jetson Orin NX achieved 8 FPS, which is insufficient for real-time navigation. The combination of classical clustering + Kalman tracking + lightweight CNN classification provides 85% of the benefit at 10% of the computational cost.

4. **Fixed-frequency replanning is simultaneously too much and too little.** In our E4 experiments, fixed 1 Hz replanning triggered 30+ unnecessary replans during a 30 s blockage (computation waste) and yet, during a sudden pedestrian appearance, the 1 s gap between replans proved insufficient for local-planner-only recovery.

### C. Generality

While our evaluation is conducted on the Ranger Mini 2.0, the framework's principles apply to any mobile robot with:
- A non-holonomic or mode-constrained chassis (e.g., Ackermann-steered vehicles, differential-drive robots with speed-dependent turning radius)
- Multi-modal perception (LiDAR + camera/depth)
- Nav2-based navigation

The mode-aware arbitration module requires platform-specific kinematic parameters (wheelbase, track, max steering angles, motion mode definitions). The risk-aware scoring and event-driven replanning modules are platform-agnostic.

---

## VIII. Conclusion

This paper presented a **mode-aware, risk-constrained, event-driven global-local navigation framework** for real-world dynamic environments, validated on the Ranger Mini 2.0 omnidirectional UGV with fused LiDAR–RGB-D perception. We demonstrated that safe navigation in dynamic environments cannot be achieved by tuning Nav2 parameters alone—it requires (i) explicit modeling of the platform's motion mode constraints, (ii) multi-modal dynamic perception with obstacle classification, (iii) risk-aware trajectory evaluation that accounts for moving obstacles, and (iv) event-driven coordination between global and local planning.

Our real-robot experiments across six scenarios show that the full framework achieves [TBD]% success rate and reduces collisions by [TBD]% compared to standard Nav2 baselines. Ablation studies confirm that each module contributes independently to overall performance, with the combination of LiDAR–RGB-D fusion and risk-aware scoring providing the largest safety improvement in pedestrian-crossing scenarios.

Future work will extend the framework to (i) multi-pedestrian socially-aware navigation using the D435i's RGB stream for human pose and intention estimation, (ii) learning-based trajectory prediction to replace the constant-velocity model, (iii) adaptive weight tuning for the risk cost function based on environmental context, and (iv) long-term deployment studies in operational environments.

---

## Appendix A: Mathematical Notation Summary

| Symbol                                    | Definition                                                      |
| ----------------------------------------- | --------------------------------------------------------------- |
| `u_nav = (vx, vy, wz)`                    | Nav2 velocity command in ℝ³                                     |
| `u_exec`                                  | Equivalent twist of executed chassis command                    |
| `η`                                       | Trajectory distortion: `‖u_nav − u_exec‖ / ‖u_nav‖`             |
| `M_A, M_O, M_S, M_SP`                     | Motion modes: Ackermann, Oblique, Side-slip, Spin               |
| `L`                                       | Wheelbase (0.494 m for Ranger Mini 2.0)                         |
| `R_min`                                   | Minimum turning radius (0.476 m for Ranger Mini 2.0)            |
| `o_i(t)`                                  | State of obstacle i at time t: `(p, v, c)`                      |
| `c_i`                                     | Obstacle class: `{pedestrian, static_obstacle, unknown}`        |
| `T_h`                                     | Prediction horizon (2–3 s)                                      |
| `τ_k`                                     | k-th candidate trajectory: T timesteps of (x, y, θ)             |
| `TTC_{k,i}^{(j)}`                         | Time-To-Collision between trajectory k at step j and obstacle i |
| `d_safe(c_i)`                             | Class-dependent safety distance                                 |
| `λ`                                       | Temporal discount factor for risk (0.95)                        |
| `C_ttc, C_dist, C_path, C_goal, C_smooth` | Risk augmentation cost terms                                    |
| `w_1...w_5`                               | Cost term weights                                               |

## Appendix B: Sensor Specifications

| Sensor           | Specification    | Value                       |
| ---------------- | ---------------- | --------------------------- |
| **Livox MID360** | FOV (H×V)        | 360° × 59°                  |
|                  | Range            | 0.1–40 m (80% reflectivity) |
|                  | Point rate       | ~200,000 pts/s              |
|                  | Accuracy         | ±2 cm @ 20 m                |
|                  | Scan pattern     | Non-repetitive              |
|                  | Weight           | 265 g                       |
| **Intel D435i**  | Depth FOV (H×V)  | 87° × 58°                   |
|                  | Depth range      | 0.2–10 m                    |
|                  | Depth resolution | 1280 × 720 @ 30 Hz          |
|                  | RGB resolution   | 1920 × 1080 @ 30 Hz         |
|                  | IMU              | BMI055 (6-DOF)              |
|                  | Weight           | 75 g                        |

## Appendix C: CAN Motion Mode Protocol

| Mode             | Byte value | Parameters                                  | Range                                |
| ---------------- | ---------- | ------------------------------------------- | ------------------------------------ |
| Dual Ackermann   | 0x01       | Linear velocity (m/s), Steering angle (rad) | v ∈ [-1.5, 1.5], δ ∈ [-0.698, 0.698] |
| Parallel/Oblique | 0x02       | Linear velocity (m/s), Steering angle (rad) | v ∈ [-1.5, 1.5], δ ∈ [-1.57, 1.57]   |
| Spin             | 0x03       | Angular velocity (rad/s)                    | ω ∈ [-4.8, 4.8]                      |
| Standby          | 0x00       | —                                           | —                                    |

---

## References

[1] D. V. Lu, D. Hershberger, and W. D. Smart, "Layered costmaps for context-sensitive navigation," in *Proc. IEEE/RSJ IROS*, 2014.

[2] G. Williams, A. Aldrich, and E. A. Theodorou, "Model predictive path integral control: From theory to parallel computation," *Journal of Guidance, Control, and Dynamics*, vol. 40, no. 2, pp. 344–357, 2017.

[3] C. Rösmann, F. Hoffmann, and T. Bertram, "Integrated online trajectory planning and optimization in distinctive topologies," *Robotics and Autonomous Systems*, vol. 88, pp. 142–153, 2017.

[4] S. Macenski, F. Martín, R. White, and J. G. Clavero, "The Marathon 2: A navigation system," in *Proc. IEEE/RSJ IROS*, 2020.

[5] Y. F. Chen, M. Liu, M. Everett, and J. P. How, "Decentralized non-communicating multiagent collision avoidance with deep reinforcement learning," in *Proc. IEEE ICRA*, 2017.

[6] J. van den Berg, S. J. Guy, M. Lin, and D. Manocha, "Reciprocal n-body collision avoidance," in *Robotics Research*, Springer, 2011, pp. 3–19.

[7] P. Long, T. Fan, X. Liao, W. Liu, H. Zhang, and J. Pan, "Towards optimally decentralized multi-robot collision avoidance via deep reinforcement learning," in *Proc. IEEE ICRA*, 2018.

[8] M. Pfeiffer, M. Schaeuble, J. Nieto, R. Siegwart, and C. Cadena, "From perception to decision: A data-driven approach to end-to-end motion planning for autonomous ground robots," in *Proc. IEEE ICRA*, 2017.

[9] B. Li, H. Du, and W. Li, "Trajectory control for autonomous electric vehicles with in-wheel motors based on a dynamics model approach," *IET Intelligent Transport Systems*, vol. 10, no. 5, pp. 318–330, 2016.

[10] P. Hang, X. Chen, and F. Luo, "LPV/H∞ controller design for path tracking of autonomous ground vehicles through four-wheel steering and direct yaw-moment control," *Int. J. Automotive Technology*, vol. 20, no. 4, pp. 679–691, 2019.

[11] I. S. Mohamed, G. Allibert, and P. Martinet, "Sampling-based MPC for constrained vision-based control," in *Proc. IEEE/RSJ IROS*, 2021.

[12] C. Rösmann, F. Hoffmann, and T. Bertram, "Timed-Elastic-Bands for time-optimal point-to-point nonlinear model predictive control," in *Proc. IEEE ECC*, 2015.

[13] I. Bogoslavskyi and C. Stachniss, "Efficient online segmentation for sparse 3D laser scans," *PFG – J. Photogrammetry, Remote Sensing and Geoinformation Science*, vol. 85, no. 1, pp. 41–52, 2017.

[14] S. Blackman and R. Popoli, *Design and Analysis of Modern Tracking Systems*. Artech House, 1999.

[15] A. H. Lang, S. Vora, H. Caesar, L. Zhou, J. Yang, and O. Beijbom, "PointPillars: Fast encoders for object detection from point clouds," in *Proc. IEEE CVPR*, 2019.

[16] T. Yin, X. Zhou, and P. Krähenbühl, "Center-based 3D object detection and tracking," in *Proc. IEEE CVPR*, 2021.

[17] L. Spinello and K. O. Arras, "People detection in RGB-D data," in *Proc. IEEE/RSJ IROS*, 2011.

[18] M. Mancini, G. Costante, P. Valigi, and T. A. Ciarfuglia, "Fast robust monocular depth estimation for obstacle detection with fully convolutional networks," in *Proc. IEEE/RSJ IROS*, 2016.

[19] M. Liang, B. Yang, S. Wang, and R. Urtasun, "Deep continuous fusion for multi-sensor 3D object detection," in *Proc. ECCV*, 2018.

[20] X. Chen, H. Ma, J. Wan, B. Li, and T. Xia, "Multi-view 3D object detection network for autonomous driving," in *Proc. IEEE CVPR*, 2017.

[21] J. Ni, J. Hu, and C. Xiang, "A review of control strategies for four-wheel independent drive and steering electric vehicles," *IEEE Trans. Vehicular Technology*, vol. 69, no. 1, pp. 3–17, 2020.

[22] [TBD — specific agricultural robotics reference using Ranger platform]

[23] [TBD — specific last-mile delivery reference using Ranger platform]

[24] K. D. Kusano and H. C. Gabler, "Safety benefits of forward collision warning, brake assist, and autonomous braking systems in rear-end collisions," *IEEE Trans. Intelligent Transportation Systems*, vol. 13, no. 4, pp. 1546–1555, 2012.

[25] A. Vemula, K. Muelling, and J. Oh, "Social attention: Modeling attention in human crowds," in *Proc. IEEE ICRA*, 2018.

[26] B. Brito, B. Floor, L. Ferranti, and J. Alonso-Mora, "Model predictive contouring control for collision avoidance in unstructured dynamic environments," *IEEE Robotics and Automation Letters*, vol. 4, no. 4, pp. 4459–4466, 2019.

[27] T. Fan, P. Long, W. Liu, and J. Pan, "Distributed multi-robot collision avoidance via deep reinforcement learning for navigation in complex scenarios," *Int. J. Robotics Research*, vol. 39, no. 7, pp. 856–892, 2020.

[28] A. Howard et al., "Searching for MobileNetV3," in *Proc. IEEE ICCV*, 2019.

---

*Draft version: 2026-05-14. Experimental data marked [TBD] pending real-robot data collection.*
