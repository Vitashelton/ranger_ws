# RangerMini v5 实车检查清单

## 1. 先确认 ROS2 网络

本机调试：

```bash
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1
ros2 daemon stop
ros2 daemon start
```

PC-Jetson 多机时不要 `ROS_LOCALHOST_ONLY=1`，并确保 CycloneDDS IP 是当前网卡。

## 2. 传感器 topic

```bash
ros2 topic hz /scan
ros2 topic hz /livox/lidar
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

## 3. 外参重点

你之前的传感器 launch 里，注释和实际 TF 可能不一致：

```text
注释：base_link -> livox_frame (0,0,0.35)
实际：x=0.30, z=0.70, pitch=0.523599
```

这个必须按实车安装重新核对。外参不准，BEV 会错位。

## 4. 安全启动顺序

```bash
# 只看输出，不驱动
enable_drive:=false

# 确认 /cmd_vel_safe 稳定后再打开
enable_drive:=true
```

## 5. 低速限幅

v5 默认 `cmd_vel_guard` 限幅：

```text
max_vx = 0.25 m/s
max_vy = 0.20 m/s
max_wz = 0.50 rad/s
```

先不要改大。
