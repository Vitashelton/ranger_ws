# Sensor Setup Guide — Ranger Mini 2.0 + MID360S + D435i

## Hardware Overview

| Component | Model | Role | Interface |
|-----------|-------|------|-----------|
| LiDAR | Livox MID360S | Primary: SLAM, mid/long-range obstacle detection (0.5–40m) | Ethernet (192.168.1.x) |
| RGB-D Camera | Intel RealSense D435i | Secondary: near-field obstacle detection, low obstacle detection (0.2–4m) | USB 3.0 |
| Chassis | Ranger Mini 2.0 | Four-wheel-steering UGV | CAN bus (gs_usb) |
| Compute | NVIDIA Jetson Orin | Onboard processing | — |

## Sensor Mounting Positions

All positions relative to `base_link` (robot geometric center, ground level z=0):

| Frame | x (m) | y (m) | z (m) | roll (°) | pitch (°) | yaw (°) |
|-------|-------|-------|-------|----------|-----------|---------|
| `base_link` → `livox_frame` | 0.30 | 0.0 | 0.70 | 0 | +30 | 0 |
| `base_link` → `camera_link` | [TBD] | 0.0 | [TBD] | 0 | [TBD] | 0 |
| `camera_link` → `camera_color_optical_frame` | 0 | 0 | 0 | -90 | 0 | -90 |
| `camera_link` → `camera_depth_optical_frame` | 0 | 0 | 0 | -90 | 0 | -90 |

**Note**: `[TBD]` values require physical measurement after mounting. See `docs/tf_calibration_checklist.md`.

## D435i Camera Mounting Recommendations

1. **Height**: Mount ~0.5–0.8 m above ground. High enough to see over the chassis front, low enough to detect low obstacles (water bottles, cables).
2. **Pitch**: Tilt down ~10–15° to capture the ground in front. This improves low obstacle detection but reduces far range.
3. **Position**: Mount on the front of the chassis, centered laterally (y=0). Distance from `base_link` center measured after mounting.

## Sensor Network Configuration

### MID360S (Ethernet)
```bash
# On Jetson, configure static IP for MID360S
sudo nmcli dev set enP8p1s0 managed no
sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.1.5/24 dev enP8p1s0
sudo ip link set enP8p1s0 up

# Verify connection
ping 192.168.1.1  # Default MID360S IP (check livox_ros_driver2 config)
```

### D435i (USB 3.0)
```bash
# Check USB connection
lsusb | grep Intel

# Verify camera
rs-enumerate-devices

# Ensure USB 3.0 bandwidth (not USB 2.0)
lsusb -t | grep -A2 "Intel"
```

## Software Dependencies

### MID360S
- `livox_ros_driver2` (installed in `/home/robot/livox_ws`)
- `pointcloud_to_laserscan` (ROS2 apt package or built from source)

### D435i
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
sudo apt install ros-humble-depth-image-proc
```

### Sensor Fusion
- `python3-numpy`, `python3-scipy` (for Kalman filter, Hungarian algorithm)
- `python3-sklearn` (for Euclidean clustering fallback)

## Launch Sequence

### 1. CAN + Chassis
```bash
sudo bash src/ranger_nav/scripts/can_bringup.sh can1 500000
```

### 2. Full System
```bash
# Option A: All sensors + Nav2
ros2 launch ranger_nav ranger_full.launch.py mode:=nav map:=/home/robot/maps/ranger_map.yaml

# Option B: Step-by-step (debugging)
ros2 launch ranger_nav ranger_base.launch.py
ros2 launch ranger_nav ranger_sensors.launch.py  # MID360S
ros2 launch ranger_nav d435i_sensor.launch.py   # D435i (new)
ros2 launch ranger_nav sensor_fusion.launch.py  # Fusion (new)
ros2 launch ranger_nav ranger_nav.launch.py map:=...
```

## Verification Checklist

```bash
# 1. TF tree
ros2 run tf2_tools view_frames
# Expected: map → odom → base_link → livox_frame
#                               base_link → camera_link → camera_*_optical_frame

# 2. MID360S point cloud
ros2 topic hz /livox/lidar     # Should be ~10 Hz
ros2 topic echo /livox/lidar --once | grep -c "Point"

# 3. D435i topics
ros2 topic hz /camera/depth/color/points   # Should be 6–15 Hz
ros2 topic hz /camera/color/image_raw

# 4. LaserScan
ros2 topic hz /scan            # Should be ~10 Hz, non-empty ranges

# 5. Obstacle detection
ros2 topic hz /obstacles_mid360
ros2 topic hz /obstacles_d435i
ros2 topic hz /fused_obstacles
```

## Config Files Reference

| Config | Purpose |
|--------|---------|
| `config/mid360_filter.yaml` | ROI, voxel downsample, ground removal params for MID360S |
| `config/d435i_filter.yaml` | ROI, cluster params for D435i |
| `config/pointcloud_to_laserscan.yaml` | MID360S → /scan conversion (existing) |
| `config/sensor_fusion.yaml` | Association threshold, confidence weights, sensor priority |

## Known Limitations

1. **MID360S blind zone**: ~0.5m around the LiDAR. Obstacles closer than this are invisible to MID360S. D435i fills this gap.
2. **D435i outdoor**: IR-based depth sensor struggles in direct sunlight. Effective range drops to ~2m outdoors.
3. **MID360S tilted mount**: 30° upward tilt means the LiDAR sees the ceiling at close range. Reduces effective near-field coverage.
4. **Sensor sync**: MID360S (10 Hz) and D435i (6–15 Hz) are not hardware-synchronized. Approximate time-sync via message timestamps in the fusion node.
