# v5 论文撰稿思路：点在哪里

## 题目建议

**面向边缘部署的室内移动机器人语义目标约束局部风险感知与共享控制方法**

或者更贴实物：

**基于 MID360S 与 D435i 的 RangerMini 室内语义目标驱动局部风险导航与共享控制方法**

## 研究问题

不要写成“我做了一个 Gazebo 仿真”或“我接了 YOLO”。

要写成：

> 在室内实验室走廊场景中，移动机器人需要根据语义目标到达指定门前区域，同时在多源传感器局部感知不稳定、人类输入可能错误、边缘算力有限的条件下，生成安全且最小干预的底盘速度指令。

## 三个核心点

### 点 1：语义目标进入控制闭环

YOLO 不是论文贡献本身。贡献是：

```text
YOLO / doorplate / door material
        ↓
semantic memory
        ↓
door-front target pose
        ↓
shared-control cost
```

语义目标从“图上标签”变成了控制器里的 `g_s`。

### 点 2：MID360S + D435i 变成局部风险接口

不是简单避障，而是：

```text
MID360S: 中远距离三维几何风险
D435i: 近场深度 / 低矮障碍 / 视觉语义
        ↓
Local BEV Risk Field R(x, y)
```

论文可写：

```text
R(x,y) = R_occ + R_clearance + R_blind + R_temporal
```

v5 先实现了 `/local_risk_grid` 接口，后续可以逐步把风险模型丰富。

### 点 3：共享控制是“最小干预”，不是遥控车

核心公式：

```text
u* = argmin_u ||u - u_h||_Q^2
       + λ_r R(τ(u))
       + λ_g D(τ(u), g_s)
       + λ_s S(u, u_prev)
```

其中：

```text
u_h：人类/上层规划期望速度
u*：安全输出速度
R(τ(u))：候选轨迹在局部风险图中的风险
g_s：语义目标门前区域
```

接管强度：

```text
I(t)= ||u*(t)-u_h(t)|| / (||u_h(t)|| + ε)
```

这就是“抢方向盘”的数学表达。

## 实验设计

### 实验 A：仿真/回放验证 shared-control

对比：

```text
Manual
Stop-only
Rule-limit
Ours
```

指标：

```text
最小障碍距离
碰撞率
接管次数
平均接管强度
速度抖动
到达时间
```

### 实验 B：真实传感器风险图验证

启动 MID360S / D435i，但不驱动底盘：

```bash
enable_drive:=false
```

指标：

```text
/local_risk_grid 更新频率
/debug/bev_image 可视化稳定性
障碍响应时间
端到端延迟
```

### 实验 C：低速实车 shared-control 验证

先小范围、低速：

```bash
enable_drive:=true
```

指标：

```text
cmd_vel_safe 与 cmd_vel_raw 差异
最小障碍距离
接管强度
通过成功率
Jetson CPU / 内存 / 延迟
```

## 章节结构

```text
第1章 绪论
  室内移动机器人语义目标到达与安全局部导航问题

第2章 系统平台与问题建模
  RangerMini 2.0 / Jetson Orin Nano / MID360S / D435i / ROS2

第3章 语义目标记忆与门前区域建模
  YOLO 观测 → semantic memory → target pose

第4章 多源传感器局部 BEV 风险表征
  MID360S + D435i → local risk grid

第5章 语义目标约束的共享控制安全过滤
  候选速度 rollout + 风险查询 + 最小干预优化

第6章 实验与分析
  仿真、真实传感器、低速实车、边缘部署指标

第7章 总结与展望
  完整四舵轮物理模型、边云协同、端到端语义策略
```
