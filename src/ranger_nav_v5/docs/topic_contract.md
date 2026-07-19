# v5 Topic Contract

## `/semantic_detections_json`

类型：`std_msgs/String`

推荐 JSON：

```json
[
  {
    "room_id": "906",
    "class": "door_front",
    "material": "glass",
    "pose": [6.0, 1.2, 0.0],
    "conf": 0.93
  }
]
```

来源：

```text
D435i RGB -> YOLO -> localizer/projection -> JSON
```

## `/local_risk_grid`

类型：`nav_msgs/OccupancyGrid`

坐标系：

```text
base_link local BEV
origin = (-width/2, -height/2)
```

含义：

```text
0   free / low risk
1-74 inflated / medium risk
75+ reject-level high risk
100 lethal obstacle
```

## `/cmd_vel_raw`

类型：`geometry_msgs/Twist`

来源：

```text
手柄 / 键盘 / 上层规划器
```

## `/cmd_vel_safe`

类型：`geometry_msgs/Twist`

由 shared-control filter 输出，不应绕过 safety guard 直接高速接底盘。

## `/intervention_score`

类型：`std_msgs/Float32`

定义：

```text
I(t) = ||u_safe - u_raw|| / (||u_raw|| + epsilon)
```
