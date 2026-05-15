# 13. ranger_nav 统一启动教程

## 13.0 前置准备（一次性）

### 13.0.1 创建 ranger_nav_ws 并编译

```bash
# 在 Jetson 上
mkdir -p /home/robot/ranger_nav_ws/src
mv /home/robot/agilex_ws/src/ranger_nav /home/robot/ranger_nav_ws/src/

source /opt/ros/humble/setup.bash
source /home/robot/livox_ws/install/setup.bash
source /home/robot/agilex_ws/install/setup.bash

cd /home/robot/ranger_nav_ws
colcon build --symlink-install
```

### 13.0.2 配置环境变量（可选，写入 ~/.bashrc）

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source /home/robot/livox_ws/install/setup.bash' >> ~/.bashrc
echo 'source /home/robot/agilex_ws/install/setup.bash' >> ~/.bashrc
echo 'source /home/robot/ranger_nav_ws/install/setup.bash' >> ~/.bashrc
```

### 13.0.3 创建地图保存目录

```bash
mkdir -p /home/robot/maps
```

### 13.0.4 安装 FAST-LIO2（3D SLAM 依赖，一次性）

FAST-LIO2 是紧耦合 LiDAR-IMU 里程计，利用 MID360 内置 IMU 做 6-DOF 位姿估计，漂移远小于纯 2D scan matching。

```bash
# 在 Jetson 上
mkdir -p /home/robot/fast_lio_ws/src
cd /home/robot/fast_lio_ws/src

# 克隆 FAST-LIO2 (ROS 2 Humble 适配版)
git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git fast_lio

# 依赖
sudo apt install -y ros-humble-pcl-ros ros-humble-tf2-eigen

# 编译
cd /home/robot/fast_lio_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select fast_lio
```

MID360 内置 IMU 通过 `livox_ros_driver2` 发布在 `/livox/imu`，无需额外传感器。

---

## 13.1 2D 建图模式（slam_toolbox）

### 终端 1：CAN + 底盘初始化

```bash
# 每次开机后执行一次
sudo modprobe gs_usb

sudo ip link set can1 down 2>/dev/null
sudo ip link set can1 type can bitrate 500000 restart-ms 100
sudo ip link set can1 up
```

### 终端 2：Livox 网络初始化

```bash
# 每次开机后执行一次
sudo nmcli dev set enP8p1s0 managed no
sudo ip addr flush dev enP8p1s0
sudo ip addr add 192.168.1.5/24 dev enP8p1s0
sudo ip link set enP8p1s0 up
ip -4 addr show enP8p1s0
```

### 终端 3：一键 SLAM

```bash
source /opt/ros/humble/setup.bash
source /home/robot/livox_ws/install/setup.bash
source /home/robot/agilex_ws/install/setup.bash
source /home/robot/ranger_nav_ws/install/setup.bash
source /home/robot/livox_ws/install/setup.bash


# 一键启动底盘 + Livox + 静态TF + 点云转 /scan + slam_toolbox + RViz
ros2 launch ranger_nav ranger_full.launch.py mode:=mapping
```

**等效于原来 5 个终端的工作。**

### 终端 4：键盘遥控（可选）

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 保存 2D 地图

```bash
# 开一个新终端
ros2 run nav2_map_server map_saver_cli -f /home/robot/maps/ranger_map
```

---

## 13.2 3D 建图模式（FAST-LIO2，推荐）

相比 2D slam_toolbox，FAST-LIO2 使用 MID360 内置 IMU 做紧耦合 LiDAR-IMU 里程计：
- **漂移极小**：~0.5%/路程（2D 方案 >3%）
- **6-DOF 姿态**：不依赖平面假设，斜坡、颠簸路面也能建图
- **3D 点云地图**：稠密，可视化更直观

### 终端 3：一键 3D SLAM

```bash
source /opt/ros/humble/setup.bash
source /home/robot/livox_ws/install/setup.bash
source /home/robot/agilex_ws/install/setup.bash
source /home/robot/fast_lio_ws/install/setup.bash
source /home/robot/ranger_nav_ws/install/setup.bash

# 一键启动底盘 + Livox + FAST-LIO2 + 3D RViz
ros2 launch ranger_nav ranger_full.launch.py mode:=mapping3d
```

### 保存 3D 点云地图

```bash
# 终端 4：调用 FAST-LIO2 的保存服务
ros2 service call /fast_lio/save_map fast_lio/srv/SaveMap

# 地图默认保存在当前目录的 PCD 文件
ls *.pcd
```

### 3D 点云地图 → 2D 栅格地图（给 Nav2 用）

Nav2 导航需要 2D 占据栅格地图（`.pgm` + `.yaml`）。建完 3D 地图后转换：

```bash
cd /home/robot/ranger_nav_ws/src/ranger_nav/scripts
pip install open3d numpy pyyaml scipy

python3 pcd_to_2d_map.py /path/to/saved.pcd ranger_3d_map \
    --resolution 0.05 \
    --z_min 0.2 \
    --z_max 2.0 \
    --dilate 2

# 输出：ranger_3d_map.pgm + ranger_3d_map.yaml
# 拷贝到地图目录
cp ranger_3d_map.* /home/robot/maps/
```

参数说明：
- `--z_min / --z_max`：截取高度范围（墙壁 0.2-2.0m，避开地面和天花板杂点）
- `--dilate 2`：障碍物膨胀 2 格（0.1m），增厚墙壁确保安全
- `--resolution 0.05`：与 Nav2 costmap 分辨率保持一致

### RViz 3D 显示

3D 模式下 RViz 会自动加载 `ranger_3d_slam.rviz`，默认显示：
- `/cloud_registered` — 实时注册点云（白色激光扫描）
- `/map` — 累计地图点云（半透明，随时间累积）
- `/path` — 里程计轨迹（绿色线）
- `/odom_lidar` — 6-DOF 姿态箭头
- TF 坐标系
- 使用鼠标左键拖拽旋转视角、滚轮缩放、中键平移。

---

## 13.3 导航模式（Nav2 Navigation）

需要先用 13.1（2D）或 13.2（3D）建好并保存地图。

```bash
source /opt/ros/humble/setup.bash
source /home/robot/livox_ws/install/setup.bash
source /home/robot/agilex_ws/install/setup.bash
source /home/robot/ranger_nav_ws/install/setup.bash

# 2D 地图导航
ros2 launch ranger_nav ranger_full.launch.py mode:=nav map:=/home/robot/maps/ranger_map.yaml

# 3D 转 2D 地图导航
ros2 launch ranger_nav ranger_full.launch.py mode:=nav map:=/home/robot/maps/ranger_3d_map.yaml
```

在 RViz2 中用 **2D Pose Estimate** 设初始位姿，用 **Nav2 Goal** 设目标点。

---

## 13.4 分步启动（调试用）

如果不想一键启动，可以分步排查：

```bash
# 底盘单独启动
ros2 launch ranger_nav ranger_base.launch.py

# 传感器单独启动（Livox + 点云转换 + 静态TF）
ros2 launch ranger_nav ranger_sensors.launch.py

# --- 2D SLAM 单独启动 ---
ros2 launch ranger_nav ranger_slam.launch.py

# --- 3D SLAM 单独启动 ---
ros2 launch ranger_nav ranger_3d_slam.launch.py

# Nav2 单独启动（依赖底盘 + 传感器 + 定位已在运行）
ros2 launch ranger_nav ranger_nav.launch.py map:=/home/robot/maps/ranger_map.yaml
```

## 13.5 调试检查

```bash
# 确认 TF 树完整（预期: map -> odom -> base_link -> livox_frame）
ros2 run tf2_tools view_frames

# 2D 模式：确认 topic
ros2 topic list | grep -E "scan|odom|map|plan|costmap|cmd_vel"

# 3D 模式：确认 topic
ros2 topic list | grep -E "livox|odom_lidar|cloud_registered|map|path"

# 确认 /scan 频率
ros2 topic hz /scan

# 确认底盘连接
ros2 topic echo /system_state --once

# 检查 /scan 是否有点（空 scan = 高度窗口不对）
ros2 topic echo /scan --once | grep -c "inf\|ranges"

# 3D 模式：检查 FAST-LIO2 是否输出 odometry
ros2 topic echo /odom_lidar --once

# 3D 模式：检查点云帧率
ros2 topic hz /cloud_registered
```

---

## 13.6 雷达外参 & 已知限制

### 当前外参

```
base_link -> livox_frame:
  x=0.30  y=0  z=0.70
  roll=0  pitch=+0.5236 (+30°)  yaw=0
```

雷达安装：正对车头，向上倾斜 30°，高度 70cm，前距 30cm。

### 对建图的影响

MID360S 垂直 FOV 为 -7°~52°。抬头 30° 后，在 base_link 坐标下能覆盖的高度约 1m~3.5m（随距离变化）。这意味着：

**2D slam_toolbox 模式：**

| 能看到 | 看不到 |
|--------|--------|
| 墙壁、门框、柱子 | 地面小障碍物（水杯、椅子腿） |
| 走廊两侧结构 | 近身低矮物体 |

所以 `/scan` 提取高度窗口设为 `min_height: 1.0 / max_height: 3.5`（而非地面高度）。建图足够，但避障不完整 — 等 Phase 3 接入 D435i 深度相机解决。

**3D FAST-LIO2 模式：**
完全利用 3D 点云，不受高度切片限制。能建稠密 3D 地图，且 IMU 融合消除 2D 投影带来的漂移。推荐用于走廊等线性场景。

---

## 13.7 参数调优

| 文件 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| `config/slam_toolbox_mapping.yaml` | `resolution` | 0.05 | 地图分辨率 (m) |
| `config/slam_toolbox_mapping.yaml` | `minimum_travel_distance` | 0.35 | 关键帧间距（设小=更密） |
| `config/slam_toolbox_mapping.yaml` | `minimum_time_interval` | 0.4 | 关键帧最小时间间隔 (s) |
| `config/slam_toolbox_mapping.yaml` | `max_laser_range` | 15.0 | 建图用激光范围 (m) |
| `config/pointcloud_to_laserscan.yaml` | `min_height` | 1.0 | 扫描切片最低高度 (m) |
| `config/pointcloud_to_laserscan.yaml` | `max_height` | 3.5 | 扫描切片最高高度 (m) |
| `config/pointcloud_to_laserscan.yaml` | `range_max` | 15.0 | 激光输出最大距离 (m) |
| `config/nav2_params.yaml` | `inflation_radius` | 0.55 | 膨胀半径（过大会堵路） |
| `config/nav2_params.yaml` | `max_vel_x` | 1.0 | 最大线速度 |
| `config/nav2_params.yaml` | `max_vel_theta` | 1.5 | 最大角速度 |
| `config/nav2_params.yaml` | `xy_goal_tolerance` | 0.15 | 目标位置容忍度 |

### FAST-LIO2 3D SLAM 参数

| 文件 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| `config/fastlio_mid360.yaml` | `point_filter_num` | 2 | 降采样：1点/N保留（越小地图越密） |
| `config/fastlio_mid360.yaml` | `max_iteration` | 4 | ICP 优化迭代次数 |
| `config/fastlio_mid360.yaml` | `filter_size_surf` | 0.3 | 曲面点体素滤波 (m) |
| `config/fastlio_mid360.yaml` | `filter_size_map` | 0.3 | 地图点体素滤波 (m) |
| `config/fastlio_mid360.yaml` | `blind` | 0.5 | 盲区距离 (m) |
| `config/fastlio_mid360.yaml` | `acc_cov` | 0.1 | 加速度计噪声协方差（越小 IMU 越信任） |
| `config/fastlio_mid360.yaml` | `gyr_cov` | 0.1 | 陀螺仪噪声协方差 |
| `config/fastlio_mid360.yaml` | `extrinsic_est_en` | false | 在线标定外参（已知时关闭） |

需要调参时直接改 `config/*.yaml`，然后 `colcon build` 或直接重启 launch。

---

## 13.8 对比：优化前 vs 优化后

| | 优化前 | 优化后 |
|------|--------|--------|
| 终端数 | 5 个 | 1 个（+ 键盘遥控） |
| `publish_odom_tf` | 原来忘开（false） | 默认 true |
| 静态 TF | 需要手动敲命令 | 自动发布 |
| slam_toolbox 报错 | Failed to compute odom pose | 修复 |
| 配置管理 | 散落在命令行参数里 | 集中在 YAML 文件 |
| 模式切换 | 手动杀进程重启 | `mode:=mapping / mapping3d / nav` 一键切换 |
| LiDAR 外参 | 未校准 | x=0.30 z=0.70 pitch=+30° |
| 2D 建图漂移 | >3%/路程 | <1%（参数调优后） |
| 3D 建图 | 不支持 | FAST-LIO2（~0.5%/路程） |
