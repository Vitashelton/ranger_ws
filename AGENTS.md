# Ranger 项目长期记忆

> 本文件只记录必须长期遵守的方向、硬件边界和验收门槛；研究路线根据真实实验结果动态更新，不把任何旧 Markdown 方案自动当作当前论文主线。门洞方案和全栈方案均为历史候选，未获当前确认不得执行。

更新时间：2026-08-30

## 当前实机路线（动态更新）

当前不冻结论文题目。优先从真实 Ranger 已暴露的问题中筛选可证伪切口：通信丢帧与时间戳异常、MID-360S 原生三维表征、以及普通行驶/自旋/蟹行等底盘模式对局部避障和导航行为的影响。它们都是待验证假设，不得提前写成论文创新。

当前状态（2026-08-30）：

- 已有可用工程数据：`baseline_clear_spin_01`、`obstacle_modes_02`；两条包的 FAST-LIO 输出约 10 Hz，但尚未完成模式段落和轨迹指标审计。
- 已确认热点同时承载 D435i 与原始 LiDAR 时会明显降频；关闭 D435i 后 LiDAR/FAST-LIO 可恢复约 10 Hz。
- 当前下一步：先完成两条 bag 的离线审计，并补齐“固定 PCD + 先验 3D 点云定位”的工程 baseline；验证重启后能恢复 `map→odom→base_link`，再做同路线/同障碍位置的受控模式对照。不先迁移 FAST-LIO、不先扩展房间/电梯导航。
- 地图与重定位是实物论文的必要前置 gate，但不是默认创新：当前实现必须明确区分 FAST-LIO 建图、先验地图匹配和 TF 桥接，不能把保存 PCD 或静态 TF 当作重定位。
- 只有当数据证明某个传感器信息或底盘模式带来可重复、可量化的差异，才进入相关工作、新颖性和最小算法原型；否则立即换假设。

已有门洞方案见 [门洞可通行性论文 Baseline](docs/DOORWAY_PAPER_BASELINE_ZH.md)，目前仅作历史候选，不是当前实验命令。已有全栈路线见 [实物全栈执行 TODO](docs/REAL_LAB_EXECUTION_TODO_ZH.md)，继续暂停。

## 实机场景与数据纪律

当前使用实验室走廊、两侧房间入口和可重复摆放的临时障碍物作为受控场景；房间级导航、电梯和固定地图只在当前假设通过后恢复。原始 MID-360S `CustomMsg`、D435i RGB-D、底盘模式和时间戳必须保留；不生成实时二维 `/scan` 作为论文主输入，也不以保存 PCD 代替原始实验数据。

所有新录制的rosbag固定写入 `/data/ranger_nav/research/`：通用实机bag放 `bags/`，门洞论文bag放 `doorway_bags/`。不得再默认写入home下的 `~/.config/ranger_nav/research/`；已有旧bag不自动移动或删除。

## 既有候选研究范围（不覆盖已冻结主线）

当前必须优先审计的论文关键词：**重定位、建图导航、表征、导航规划**。这些关键词只用于约束问题搜索范围，不预设现有 EventSlice、LLM、TCA-BEV 或任何已写 Markdown 路线为最终论文主线。

围绕一个问题域寻找可发表且能落到 Ranger 实机的问题：语言导航或长时任务执行过程中，环境变化、目标变化或信息不完整时，机器人如何选择继续执行、验证、局部修复或重新规划。

当前只冻结问题域和硬件边界，不冻结 EventSlice、论文题目、主仿真平台或最终算法。EventSlice 0.2 是候选基线/原型，不自动等于论文创新。

候选算法链路：

```text
语言 -> JSON任务图 -> 事实证据/事件 -> 依赖切片 -> 局部修复 -> 导航执行
```

“持久化数字孪生记忆＋远程监督更新”只记录为候选架构，不是已确定主线。若后续验证，它应被收敛为：版本化的任务相关事实、远程候选更新、确定性校验/合并/回滚，以及静态地图和计划的增量复用；不先做大型三维孪生、任意远程覆盖、全自主多Agent或新的视觉/SLAM算法。数字孪生不直接“思考”，Agent只负责提出更新或决策，机器人传感器负责验证。

LLM只负责一次性语言编译和受约束的候选计划生成；不输出 `/cmd_vel`，不绕过任务依赖验证器，也不能读取仿真真值。新颖性和可验证性闸门未通过前，不把任何候选机制升级成“主算法”，也不跑大批量实验。

## 硬件边界

目标硬件固定为：RangerMini 2.0、Livox MID-360S、Intel RealSense D435i、Jetson Orin Nano。

- `ranger_nav`、FAST-LIO 和 Nav2 是定位、避障和执行基础，默认固定，不作为本文新算法。
- MID-360S 是 3D 激光雷达。主链路保留点云：Livox `CustomMsg` 或 `sensor_msgs/PointCloud2`，例如驱动原始点云和 `/nav/points`；不要把它默认转换成二维 `/scan` 再声称使用了MID-360S三维感知。
- MID-360S 实机 baseline 固定使用 Jetson 上的 `/home/robot/livox_ws/src/livox_ros_driver2/config/MID360s_config.json`，不得替换为 `MID360_config.json`。已跑通参数为：雷达 `192.168.1.195`、Jetson 雷达网口 `192.168.1.5`、`xfer_format=1`、`/livox/lidar` 为 `livox_ros_driver2/msg/CustomMsg`、`/livox/imu` 为 `sensor_msgs/msg/Imu`、发布频率 `10 Hz`、帧 `livox_frame`；JSON 中 `lidar_type=8`、设备节名为 `Mid360s`。
- FAST-LIO 源码和配置中的 `MID360` 枚举、`mid360_handler`、`fastlio_mid360.yaml` 等属于上游软件内部兼容命名，不代表硬件型号可以写成 MID-360，也不得据此换用普通 MID360 驱动配置。
- `sensor_msgs/LaserScan` 只允许用于旧版Nav2兼容、调试或明确标注的二维对照实验，不能成为论文主传感器接口。
- D435i 只在确有必要时提供RGB/深度事实证据；不开展新的完整视觉融合、BEV或目标检测论文。
- Jetson Orin Nano上记录端到端延迟、CPU/GPU/内存和LLM/API等待开销；不以桌面机结果冒充实机实时性。

实机部署分工固定为：Jetson只启动Ranger底盘、MID-360S和D435i；当前受控实验的 FAST-LIO、Nav2、任务/Agent 层和 RViz 放 PC。因热点带宽导致的降频先通过传感器录制配置和通信诊断处理；未经能力探针和用户确认，不把 FAST-LIO 迁移到 Jetson。

实机ROS2通信固定使用 `ROS_DOMAIN_ID=24`、PC热点地址 `172.20.10.3` 和 Jetson热点地址 `172.20.10.4`；PC source `~/ros2_comm_pc.sh`，Jetson source `~/ros2_comm_jetson2.sh`。旧 `192.168.3.0/24` 配置不再使用。

## Python运行时隔离

- ROS2 Humble、MID-360S驱动、Nav2和Ranger节点使用系统 Python 3.10；Habitat-Sim 使用 `ranger_habitat` 的 Python 3.9。
- 不在同一个 Python 进程中同时导入 `rclpy` 和 `habitat_sim`。两者需要交互时使用独立进程和 JSON/socket 等进程间接口。
- Habitat必须从不加载ROS工作区的shell启动，至少清理 `PYTHONPATH`、`PYTHONHOME`、`AMENT_PREFIX_PATH`、`COLCON_PREFIX_PATH`、`CMAKE_PREFIX_PATH` 及 ROS 相关变量；ROS终端不得激活 `ranger_habitat`。

## 平台分工（待能力探针后确定）

- `Habitat-Sim + LHPR-VLN/HM3D`：候选批量实验平台；先验证动态对象、观测证据、长时任务和批量评测是否真的支持目标问题。
- `rangermini_dynamic_semantic`：Gazebo与ROS接口、实验室近似场景和动态事件的候选验证平台。
- RangerMini实机：证明候选抽象接口能接入真实点云、RGB-D、定位和Nav2；不要求底层动作与Habitat相同。
- AI2-THOR/ProcTHOR及其他外部场景：只有能回答研究问题时才启用，不因数据量或新旧程度自动升级为主平台。

LHPR只提供任务、场景和轨迹工作负载。自定义事件协议应称为“基于LHPR的动态事件扩展”，不能把它写成官方动态修复基准。低层移动/转向动作数和高层任务节点数必须分开统计，不能把平均约150个低层动作写成150个任务依赖节点。不得训练LHPR官方VLN模型，也不宣称本文是新的视觉语言导航模型。

## MVP

MVP只包含：

1. 6–10个可映射到Ranger的任务原语：`navigate`、`inspect`、`confirm`、`wait`、`return`、`stop`等；
2. 一个可校验的JSON任务图和依赖关系；
3. 无关、局部、连锁三类事件，以及必要的恢复事件；
4. 事件证据、依赖切片、最小可行局部修复和统一日志；
5. 固定的低层导航执行器，不在执行器里偷偷重规划。

MVP通过标准：同一事件下，无关变化不触发修复；局部变化只改受影响子图；连锁变化覆盖完整依赖闭包；算法不读取在线仿真真值；Habitat和Gazebo使用同一任务图、证据、修复器和代价接口。

## 仿真门控

按以下顺序推进，任何一关失败都暂停扩展：

1. **问题与新颖性门**：完成问题定义、相关工作对照和最小可证伪原型。若只是“依赖图＋局部修复”的重述，停止并改题。
2. **平台能力探针**：分别确认实验室Gazebo、Habitat和Ranger能提供什么观测、事件、执行反馈和评测量，再选择主实验平台。
3. **纯Python门**：对候选机制通过任务图、非法依赖、效果撤销、预算变化和事件证据测试；在线算法只能收到可用证据。
4. **单回合门**：在至少一个与实验室或真实任务对应的后端跑通任务、事件、执行反馈和日志；不允许控制器偷偷替代候选算法。
5. **主实验门**：先跑少量种子验收协议，再做多任务、多布局、事件可观测性和预算扫描；不得用少量模板乘大量种子冒充任务多样性。
6. **实机门**：最后在RangerMini上低速执行2–3个目标点任务，验证候选接口、事件证据、安全停止和资源占用。

## 统一接口和实验纪律

算法核心只依赖：`TaskGraph`、`FactBelief/EventEvidence`、`TaskPrimitive`、`ExecutionFeedback`、`QueryBudget/CostModel`。平台差异放在适配器中，算法包内不得按 `if habitat` / `if ranger` 分叉。

所有方法共享相同的初始计划、低层导航器、传感器前端和执行器。主实验至少区分无关、局部、连锁事件，并报告成功率、无效/漏触发率、验证与修复延迟、LLM调用/Token/API等待、修改节点数、前缀保留率和完成时间。

事件生成规则、任务集合、预算、随机种子和日志格式必须在看结果前固定。结果必须明确标注为符号验证、Gazebo仿真、Habitat主实验或真实硬件，不能用内部演示数字代替论文证据。

## 明确不做

暂不做 RangerWM、BEV-LLM迁移、TCA-BEV新网络、YOLO、新SLAM、新局部规划器、开放式主动探索、能耗建模和机械臂操作；不把TCA-BEV发展成第二条主算法线。

## 工作区纪律

- 保留用户已有修改；修改前先检查目标文件，使用小范围补丁。
- 不执行 `git reset --hard`、`git checkout --` 或清理不相关文件。
- 论文级结论必须来自可复现实验；仿真结果和实机结果分开记录。
- 任何大规模实现或实验都必须先通过上面的门控，并得到用户确认。

## 动态 TODO（每次真实实验后更新）

- [ ] **Ranger Mini 2.0 运动学适配优先于调参**：明确区分标准 `dwb_core::DWBLocalPlanner` 的理想 Twist 轨迹与底盘实际的 Dual Ackermann、Parallel/Oblique、Spin 模式；在没有验证执行轨迹前，不把擦碰归因于 critic 权重或 `sim_time`。
- [ ] **测量并替换机器人碰撞模型**：测量载荷状态下 `base_link` 到车体最外轮廓的长、宽、前后/左右偏置和自旋包络；将 global/local costmap 的 `robot_radius` 更新为实测多边形 `footprint`，再重新确定 `inflation_radius`。
- [ ] **验证 `/cmd_vel` 到底盘模式的映射**：用低速直行、定半径转弯、原地自旋、平移/蟹行和模式切换测试，记录 `/cmd_vel`、`/odom`、`/motion_state`、转向/速度反馈，检查指令轨迹与执行轨迹的偏差、最小转弯半径和切换瞬态。
- [ ] **建立模式感知的局部避障基线**：先保留标准 DWB 作为对照，按实际底盘模式计算/验证 swept footprint 和碰撞距离；只有执行轨迹证据稳定后，才实现模式感知轨迹采样或代价修正，不直接把它写成论文创新。
- [ ] **运行时核验 Nav2 参数是否生效**：启动后读取 `controller_server` 参数，确认 `decel_lim_*`、`max_vel_*`、`sim_time`、costmap footprint/inflation 和 VoxelLayer 高度配置实际被加载；同时区分全局规划调度、局部控制频率、costmap 更新频率和 RViz 发布频率。
- [ ] **完成受控擦碰验收**：在相同路线、障碍位置和低速限制下，对比标准 DWB、修正 footprint/膨胀参数以及模式感知候选；至少记录最小障碍距离、擦碰次数、模式段、轨迹误差、停止/恢复延迟和资源占用。
- [ ] 审计 `baseline_clear_spin_01` 和 `obstacle_modes_02`：确认 `/motion_state` 的真实模式段、轨迹连续性、LiDAR 帧率和点云丢失区间。
- [ ] 在不启动门洞 pilot 的前提下，完成一次同路线/同障碍位置的普通行驶、 自旋和蟹行受控对照；只记录可测差异，不提前宣称创新。
- [ ] 若模式差异不稳定，优先检查底盘遥控模式、热点丢包和时间戳；不继续堆叠规划器、LLM 或视觉模块。
- [ ] 只有当前述差异可复现，才冻结论文问题、选择 baseline 并实现最小算法原型。

- [ ] 仅当后续需要 FAST-LIVO 或把相机语义投影到雷达/地图坐标时，才使用现有 FAST-Calib ArUco 板完成 D435i–MID-360S 外参标定。
- [ ] 当前不把 D435i 作为建图传感器；先作为独立 RGB/Depth 观测记录，只有研究问题需要时再做标定与时间偏移实验。
- [ ] 若后续需要用图像和机器人位姿做时空关联，再单独评估图像时间偏移；在此之前不把相机标定工作扩展成独立研究线。
