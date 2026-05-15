# TF Calibration Checklist — Ranger Mini 2.0

## Required TF Frames

```
map (slam_toolbox / FAST-LIO2 publishes map→odom)
└── odom (ranger_base_node publishes odom→base_link)
    └── base_link (robot geometric center, ground level)
        ├── livox_frame (MID360S LiDAR)
        │   └── livox_imu_frame (MID360S built-in IMU, co-located)
        └── camera_link (D435i mounting bracket)
            ├── camera_color_optical_frame
            ├── camera_depth_optical_frame
            ├── camera_color_frame
            └── camera_depth_frame
```

## Static Transforms to Publish

### 1. base_link → livox_frame (MID360S)

```
Current (approximate, from ranger_sensors.launch.py):
  x = 0.30 m    (forward from robot center)
  y = 0.00 m    (centered laterally)
  z = 0.70 m    (height above ground)
  roll  = 0.0
  pitch = +0.5236 rad (+30°, upward tilt)
  yaw   = 0.0
```

**Calibration needed**:
- [ ] Measure x: distance from base_link center to LiDAR center (along robot x-axis)
- [ ] Measure y: lateral offset (should be ~0 if centered)
- [ ] Measure z: height from ground to LiDAR optical center
- [ ] Verify pitch: 30° tilt angle (check mounting bracket)
- [ ] Verify yaw: LiDAR should face forward (0°)

**Method**: Physical measurement with tape measure. Verify by checking ground plane in point cloud.

### 2. base_link → camera_link (D435i)

```
[TBD] values:
  x = [TBD] m    (forward from robot center)
  y = 0.00 m     (centered laterally)
  z = [TBD] m    (height above ground)
  roll  = 0.0
  pitch = [TBD] rad (slight downward tilt recommended, ~10-15°)
  yaw   = 0.0
```

**Calibration needed**:
- [ ] Measure x: distance from base_link center to camera mount (along robot x-axis)
- [ ] Measure y: lateral offset (should be ~0 if centered)
- [ ] Measure z: height from ground to camera lens
- [ ] Measure pitch: downward tilt angle
- [ ] Verify yaw: camera should face forward

### 3. camera_link → camera_color_optical_frame

```
Standard ROS convention for Intel RealSense:
  x = y = z = 0
  roll  = -1.5708 rad (-90°)  [rotation about x]
  pitch = 0.0
  yaw   = -1.5708 rad (-90°)  [rotation about z]
```

This is the standard `_optical_frame` transform. The D435i URDF typically publishes this automatically via `realsense2_camera` node with `publish_tf: true`. If using manual `static_transform_publisher` instead, use the values above.

### 4. camera_link → camera_depth_optical_frame

Same transform as color_optical_frame (D435i color and depth are registered):
```
  x = y = z = 0
  roll  = -1.5708
  pitch = 0.0
  yaw   = -1.5708
```

**Note**: If `realsense2_camera` publishes TF, these may be handled automatically. Check with `ros2 run tf2_tools view_frames`.

## Calibration Procedure

### Step 1: Physical Measurement
1. Mark base_link center on the floor (midpoint between 4 wheels)
2. Measure x, y, z offsets to LiDAR center
3. Measure x, y, z offsets to camera lens center
4. Measure LiDAR pitch angle (use digital angle finder or phone app)
5. Document all values

### Step 2: Coarse Verification in RViz
```bash
# Start all TF publishers
ros2 launch ranger_nav ranger_sensors.launch.py
ros2 launch ranger_nav d435i_sensor.launch.py

# Visualize TF tree
ros2 run tf2_tools view_frames
# Open frames.pdf

# In RViz:
# - Add TF display
# - Add PointCloud2 for /livox/lidar
# - Add PointCloud2 for /camera/depth/color/points
# - Verify pointclouds align with TF frames
```

### Step 3: Ground Plane Check
```
Check that /livox/lidar ground plane is at z ≈ 0 in base_link frame.
If ground appears at non-zero z: adjust z offset in static TF.
```

### Step 4: Overlap Check
```
Place a calibration target (e.g., box, pole) at ~1.5m in front of robot.
Check that the target appears in BOTH:
  - MID360S pointcloud (after pointcloud_to_laserscan)
  - D435i depth pointcloud
If positions differ significantly (> 5cm), re-measure or re-calibrate extrinsics.
```

## Implementation: ranger_sensors.launch.py (Update)

```python
# Add D435i static TF to ranger_sensors.launch.py:

# Static TF: base_link -> camera_link
camera_static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='camera_static_tf',
    arguments=[
        '--x', '0.35',        # [TBD] — measure
        '--y', '0.0',
        '--z', '0.60',        # [TBD] — measure
        '--roll', '0.0',
        '--pitch', '-0.2618', # [TBD] — -15° downward
        '--yaw', '0.0',
        '--frame-id', 'base_link',
        '--child-frame-id', 'camera_link',
    ],
)
```

## Implementation: d435i_sensor.launch.py (New)

```python
# D435i camera launch: realsense2_camera + optional static TFs
# If realsense2_camera publishes TFs automatically (enable_tf: true),
# camera_link → optical_frame TFs are handled by the driver.
# Otherwise, publish them manually.
```

## Things NOT to Do

1. **Do NOT publish map→odom statically**. This is the SLAM/localization output and must be dynamic.
2. **Do NOT publish odom→base_link statically**. ranger_base_node publishes this from wheel odometry.
3. **Do NOT use uncalibrated extrinsics** in your paper. Mark all pre-calibration values as [TBD].
4. **Do NOT use the same TF values for different mounting setups**. If you remount sensors, re-measure.
