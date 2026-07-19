# RangerMini V5：实物接口优先的语义目标 + 局部风险 + 共享控制层

这版不再继续卷“仿真世界好不好看”，而是把 v4.x 的 shared-control 思路接到你的真实硬件链路：

```bash
ros2 launch ranger_nav ranger_realtime_bringup.launch.py \
  use_mid360:=true \
  use_d435i:=true \
  use_semantic:=true \
  use_shared_control:=true
```

## 1. v5 的定位

v5 是 **real-sensor-ready interface layer**：

```text
ranger_base.launch.py
    → RangerMini 2.0 底盘 /odom /tf /cmd_vel

ranger_sensors.launch.py
    → MID360S /livox/lidar /scan
    → D435i RGB / depth

ranger_nav_v5
    → /local_risk_grid
    → /semantic_target_pose
    → /cmd_vel_safe
    → CSV / RViz debug
```

核心原则：

```text
仿真只做验证；
真实传感器负责风险图；
真实底盘负责闭环；
shared-control filter 在仿真和实物中保持同一套接口。
```

## 2. 推荐安装方式

把 `ranger_nav_v5` 作为新 package 放进同一个 workspace：

```bash
cd ~/ranger_ws/src
unzip ~/Downloads/rangermini_v5_real_sensor_ready.zip
cp -r rangermini_v5_real_sensor_ready/ranger_nav_v5 .
```

如果你想保留你喜欢的命令风格，把 drop-in launch 放进现有 `ranger_nav`：

```bash
cp rangermini_v5_real_sensor_ready/dropin_for_existing_ranger_nav/launch/ranger_realtime_bringup.launch.py \
   ~/ranger_ws/src/ranger_nav/launch/
```

然后清缓存并编译：

```bash
cd ~/ranger_ws
rm -rf build/ranger_nav_v5 install/ranger_nav_v5 log
colcon build --packages-select ranger_nav_v5 ranger_nav --symlink-install
source install/setup.bash
```

## 3. 安全测试顺序

### 3.1 先不接底盘，只看算法输出

```bash
ros2 launch ranger_nav ranger_realtime_bringup.launch.py \
  use_base:=false \
  use_mid360:=true \
  use_d435i:=true \
  use_semantic:=true \
  use_shared_control:=true \
  enable_drive:=false
```

检查：

```bash
ros2 topic echo /local_risk_grid --once
ros2 topic echo /cmd_vel_safe
ros2 topic echo /intervention_score
ros2 topic echo /semantic_target_pose
```

### 3.2 没有 YOLO 时，用 semantic stub 测链路

```bash
ros2 launch ranger_nav ranger_realtime_bringup.launch.py \
  use_base:=false \
  use_mid360:=false \
  use_d435i:=false \
  use_semantic:=true \
  use_semantic_stub:=true \
  use_shared_control:=true \
  enable_drive:=false
```

### 3.3 真正驱动底盘前

先让车架空或低速空旷地面测试：

```bash
ros2 launch ranger_nav ranger_realtime_bringup.launch.py \
  use_base:=true \
  use_mid360:=true \
  use_d435i:=true \
  use_semantic:=true \
  use_shared_control:=true \
  enable_drive:=false
```

确认 `/cmd_vel_safe` 稳定后，最后才开：

```bash
enable_drive:=true
```

`enable_drive:=true` 会启动 `cmd_vel_guard`，把 `/cmd_vel_safe` 限幅后发布到 `/cmd_vel`。

## 4. Topic 合同

输入：

```text
/scan                         MID360S 转 LaserScan 后的快速风险代理
/livox/lidar                  MID360S 原始/去畸变点云，可选
/odom                         RangerMini 底盘里程计
/cmd_vel_raw                  人类手柄/键盘/上层规划器期望速度
/semantic_detections_json     YOLO + localizer 输出，JSON，可选
```

输出：

```text
/local_risk_grid              局部 BEV 风险图
/debug/bev_image              rqt_image_view 可看的风险图
/debug/risk_markers           RViz 风险点
/semantic_target_pose         目标房间门前区域
/cmd_vel_safe                 shared-control 安全速度
/intervention_score           接管强度 I(t)
/debug/rangermini2_steer_modules  RangerMini 2.0 四转向模块可视化
```

## 5. 系统框图

```mermaid
flowchart LR
    A[MID360S /livox/lidar, /scan] --> R[Local BEV Risk Node]
    B[D435i depth/RGB] --> R
    C[D435i RGB + YOLO] --> S[Semantic Memory]
    D[RangerMini /odom, /tf] --> R
    D --> F[Shared-Control Filter]
    R --> F
    S --> F
    H[Human / Planner /cmd_vel_raw] --> F
    F --> O[/cmd_vel_safe]
    O --> G{enable_drive?}
    G -- false --> V[RViz / CSV only]
    G -- true --> M[cmd_vel_guard -> /cmd_vel -> RangerMini 2.0]
```

## 6. 论文主线

v5 可以这样写：

> 本文首先在仿真环境中验证语义目标约束共享控制框架的闭环逻辑，随后将仿真障碍替换为由 MID360S 点云与 D435i 深度图构建的实时局部 BEV 风险图，并接入 RangerMini 2.0 底盘驱动。由于仿真与实物系统共享 `/local_risk_grid`、`/semantic_target_pose` 与 `/cmd_vel_safe` 接口，因此核心共享控制器无需重写即可完成从仿真到实物的迁移。

