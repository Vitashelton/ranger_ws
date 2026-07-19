#!/usr/bin/env python3
"""rangerwm_mpc/mpc_node —— 世界模型引导的动作重排 (2-5Hz), 硬件适配版.

订阅:
  /odom          (nav_msgs/Odometry)      底盘里程计, 50Hz
  /scan          (sensor_msgs/LaserScan)  前向障碍 (pointcloud_to_laserscan), 10Hz
  /goal_pose     (geometry_msgs/PoseStamped) 导航目标 (RViz "2D Goal Pose" 或 Nav2)

发布:
  /policy/mode_aware_cmd  (rangerwm_msgs/ModeAwareCmd)  选定动作首个 chunk
  /rangerwm/imagined      (rangerwm_msgs/ImaginedRollout)  调试输出

复用 rangerwm_planning.mpc (与 sim demo 同一套逻辑), WM 后端可切换:
  wm_backend = "analytic"  -> AnalyticWorldModel (无 torch, 可立即跑)
  wm_backend = "learned"   -> 加载 TRT/ONNX/torch world model (部署)

障碍处理:
  /scan -> body-frame obstacles -> 每次 MPC 迭代时用当前 state 转到 world 坐标
  超时 1.0s 无 scan -> 空障碍 + warning
"""
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from rangerwm_msgs.msg import ModeAwareCmd, ImaginedRollout
from rangerwm_planning import mpc as MPC
from rangerwm_planning.analytic_wm import AnalyticWorldModel


def _quaternion_to_yaw(q):
    """ros2 四元数 -> yaw."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class MPCNode(Node):
    def __init__(self):
        super().__init__("rangerwm_mpc")

        # ---- 参数声明 (默认值从 bringup_params.yaml / Nav2 确认) ----
        d = self.declare_parameter
        d("rate_hz", 3.0)
        d("wm_backend", "analytic")
        d("H", 10)
        d("exec_len", 3)
        d("dt", 0.1)

        # 障碍参数
        d("obstacles.source_topic", "/scan")
        d("obstacles.min_range", 0.3)
        d("obstacles.max_range", 15.0)
        d("obstacles.robot_radius", 0.35)
        d("obstacles.obstacle_radius", 0.10)
        d("obstacles.grid_cell_size", 0.15)
        d("obstacles.scan_timeout_s", 1.0)

        # 目标参数
        d("goal.topic", "/goal_pose")
        d("goal.default_x", 5.0)
        d("goal.default_y", 0.0)
        d("goal.xy_tolerance", 0.3)

        # MPC 参数
        d("mpc.v_grid", [0.2, 0.4, 0.7, 1.0])
        d("mpc.spin_grid", [-1.5, 1.5])
        d("mpc.n_random", 12)
        d("mpc.weights.goal", 1.0)
        d("mpc.weights.collision", 2.5)
        d("mpc.weights.smooth", 0.1)
        d("mpc.weights.uncertainty", 0.0)

        # ---- 状态 ----
        self.state = (0.0, 0.0, 0.0)          # (x, y, yaw) in odom frame
        self.body_obstacles = []               # [(body_x, body_y, r)] in base_link frame
        self.last_scan_t = 0.0
        self.goal = (5.0, 0.0)                # default goal (world frame)
        self.goal_received = False

        self.rng = np.random.default_rng(0)

        # ---- WM 后端 ----
        self._init_wm()

        # ---- 订阅 ----
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(
            LaserScan, self._p("obstacles.source_topic"), self.on_scan, 5)
        self.create_subscription(
            PoseStamped, self._p("goal.topic"), self.on_goal, 10)

        # ---- 发布 ----
        self.pub = self.create_publisher(ModeAwareCmd, "/policy/mode_aware_cmd", 10)
        self.pub_dbg = self.create_publisher(ImaginedRollout, "/rangerwm/imagined", 5)

        # ---- 定时器 ----
        self.dt_timer = 1.0 / float(self._p("rate_hz"))
        self.create_timer(self.dt_timer, self.on_timer)

        self.get_logger().info(
            f"MPC node ready (backend={self._p('wm_backend')}, "
            f"Hz={self._p('rate_hz')}, robot_r={self._p('obstacles.robot_radius')})")

    # ----------------------------------------------------------------
    #  参数快捷方法
    # ----------------------------------------------------------------
    def _p(self, key, default=None):
        """带默认值的参数获取 (处理嵌套 key 'a.b.c')."""
        try:
            return self.get_parameter(key).value
        except Exception:
            return default

    # ----------------------------------------------------------------
    #  WM 初始化
    # ----------------------------------------------------------------
    def _init_wm(self):
        robot_r = float(self._p("obstacles.robot_radius", 0.35))
        if self._p("wm_backend") == "analytic":
            self.wm = AnalyticWorldModel(robot_radius=robot_r, d_soft=0.45)
        else:
            self.get_logger().warn("learned WM 未实接, 退回 analytic。")
            self.wm = AnalyticWorldModel(robot_radius=robot_r, d_soft=0.45)

    # ----------------------------------------------------------------
    #  回调
    # ----------------------------------------------------------------
    def on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = _quaternion_to_yaw(q)
        self.state = (msg.pose.pose.position.x,
                      msg.pose.pose.position.y,
                      yaw)

    def on_scan(self, msg: LaserScan):
        """LaserScan -> body-frame obstacles with grid downsampling."""
        self.body_obstacles = self._scan_to_body_obstacles(msg)
        self.last_scan_t = time.time()

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_received = True
        self.get_logger().info(
            f"Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    # ----------------------------------------------------------------
    #  MPC 主循环
    # ----------------------------------------------------------------
    def on_timer(self):
        # 障碍超时检查
        now = time.time()
        if now - self.last_scan_t > float(self._p("obstacles.scan_timeout_s", 1.0)):
            if self.last_scan_t > 0:  # 至少收过一次 scan
                self.get_logger().warn(
                    f"Scan timeout ({now - self.last_scan_t:.1f}s), using empty obstacles.",
                    throttle_duration_sec=2.0)
            world_obstacles = []
        else:
            world_obstacles = self._body_to_world_obstacles()

        # 目标: 优先 /goal_pose, 否则用默认值
        if not self.goal_received:
            self.goal = (float(self._p("goal.default_x", 5.0)),
                         float(self._p("goal.default_y", 0.0)))

        # 构建 MPC 配置
        cfg = dict(
            H=int(self._p("H", 10)),
            dt=float(self._p("dt", 0.1)),
            exec_len=int(self._p("exec_len", 3)),
            v_grid=list(self._p("mpc.v_grid", [0.2, 0.4, 0.7, 1.0])),
            spin_grid=list(self._p("mpc.spin_grid", [-1.5, 1.5])),
            n_random=int(self._p("mpc.n_random", 12)),
            weights=dict(
                goal=float(self._p("mpc.weights.goal", 1.0)),
                collision=float(self._p("mpc.weights.collision", 2.5)),
                smooth=float(self._p("mpc.weights.smooth", 0.1)),
                uncertainty=float(self._p("mpc.weights.uncertainty", 0.0)),
            ),
        )

        # MPC 规划
        chunk, dbg = MPC.plan(self.state, self.wm, world_obstacles, self.goal, cfg, self.rng)

        # 发布首步动作
        if chunk:
            mode, p1, p2 = chunk[0]
            m = ModeAwareCmd()
            m.header.stamp = self.get_clock().now().to_msg()
            m.mode = int(mode)
            m.param1 = float(p1)
            m.param2 = float(p2)
            self.pub.publish(m)

    # ----------------------------------------------------------------
    #  LaserScan -> body-frame obstacles
    # ----------------------------------------------------------------
    def _scan_to_body_obstacles(self, msg: LaserScan):
        """将 LaserScan 转为 base_link 系下的 (x, y, r) 障碍列表, 做 grid 降采样."""
        min_r = float(self._p("obstacles.min_range", 0.3))
        max_r = float(self._p("obstacles.max_range", 15.0))
        obs_r = float(self._p("obstacles.obstacle_radius", 0.10))
        cell = float(self._p("obstacles.grid_cell_size", 0.15))

        grid = {}  # (ix, iy) -> [(x, y), ...]
        angle = msg.angle_min
        for r in msg.ranges:
            if min_r < r < max_r:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                ix = int(x / cell)
                iy = int(y / cell)
                grid.setdefault((ix, iy), []).append((x, y))
            angle += msg.angle_increment

        # 每格取中位数
        obstacles = []
        for (ix, iy), pts in grid.items():
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            obstacles.append((
                float(np.median(xs)),
                float(np.median(ys)),
                obs_r,
            ))
        return obstacles

    # ----------------------------------------------------------------
    #  body -> world 坐标转换
    # ----------------------------------------------------------------
    def _body_to_world_obstacles(self):
        """用当前 state 把 body-frame 障碍转到 world (odom frame)."""
        x0, y0, yaw = self.state
        c, s = math.cos(yaw), math.sin(yaw)
        world = []
        for bx, by, br in self.body_obstacles:
            wx = x0 + bx * c - by * s
            wy = y0 + bx * s + by * c
            world.append((wx, wy, br))
        return world


def main():
    rclpy.init()
    node = MPCNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
