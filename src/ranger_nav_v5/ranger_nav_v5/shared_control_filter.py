import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32, Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SharedControlFilter(Node):
    """Risk-aware minimal-intervention shared-control filter.

    Inputs:
      /cmd_vel_raw              human / upper planner expected velocity
      /local_risk_grid          local BEV risk map in base_link
      /semantic_target_pose     target pose in odom/map frame
      /odom                     robot pose

    Outputs:
      /cmd_vel_safe
      /intervention_score
      /debug/shared_control_markers
      /debug/rangermini2_steer_modules
      /debug/steer_module_states
    """

    def __init__(self):
        super().__init__('shared_control_filter')

        self.human_cmd_topic = self.declare_parameter('human_cmd_topic', '/cmd_vel_raw').value
        self.safe_cmd_topic = self.declare_parameter('safe_cmd_topic', '/cmd_vel_safe').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.risk_grid_topic = self.declare_parameter('risk_grid_topic', '/local_risk_grid').value
        self.semantic_target_topic = self.declare_parameter('semantic_target_topic', '/semantic_target_pose').value

        self.loop_hz = float(self.declare_parameter('loop_hz', 20.0).value)
        self.horizon_s = float(self.declare_parameter('horizon_s', 1.2).value)
        self.dt = float(self.declare_parameter('rollout_dt', 0.15).value)
        self.max_vx = float(self.declare_parameter('max_vx', 0.35).value)
        self.max_vy = float(self.declare_parameter('max_vy', 0.30).value)
        self.max_wz = float(self.declare_parameter('max_wz', 0.70).value)
        self.max_acc = float(self.declare_parameter('max_acc', 0.45).value)

        self.vx_samples = [float(v) for v in self.declare_parameter('vx_samples', [-0.1, 0.0, 0.1, 0.2, 0.3]).value]
        self.vy_offsets = [float(v) for v in self.declare_parameter('vy_offsets', [-0.18, -0.09, 0.0, 0.09, 0.18]).value]
        self.wz_offsets = [float(v) for v in self.declare_parameter('wz_offsets', [-0.35, 0.0, 0.35]).value]

        self.w_intent = float(self.declare_parameter('w_intent', 2.2).value)
        self.w_risk = float(self.declare_parameter('w_risk', 8.0).value)
        self.w_goal = float(self.declare_parameter('w_goal', 1.2).value)
        self.w_progress = float(self.declare_parameter('w_progress', 0.8).value)
        self.w_smooth = float(self.declare_parameter('w_smooth', 0.3).value)
        self.risk_reject_threshold = float(self.declare_parameter('risk_reject_threshold', 75.0).value)

        self.robot_length = float(self.declare_parameter('robot_length_m', 0.78).value)
        self.robot_width = float(self.declare_parameter('robot_width_m', 0.58).value)
        self.footprint_margin = float(self.declare_parameter('footprint_margin_m', 0.08).value)
        self.wheelbase = float(self.declare_parameter('wheelbase_m', 0.56).value)
        self.track = float(self.declare_parameter('track_m', 0.46).value)
        self.publish_debug_markers = bool(self.declare_parameter('publish_debug_markers', True).value)

        self.raw = Twist()
        self.last_safe = Twist()
        self.odom: Optional[Odometry] = None
        self.target: Optional[PoseStamped] = None
        self.grid: Optional[OccupancyGrid] = None

        self.create_subscription(Twist, self.human_cmd_topic, self.raw_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(OccupancyGrid, self.risk_grid_topic, self.grid_cb, 10)
        self.create_subscription(PoseStamped, self.semantic_target_topic, self.target_cb, 10)

        self.safe_pub = self.create_publisher(Twist, self.safe_cmd_topic, 10)
        self.intervention_pub = self.create_publisher(Float32, '/intervention_score', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/debug/shared_control_markers', 10)
        self.steer_marker_pub = self.create_publisher(MarkerArray, '/debug/rangermini2_steer_modules', 10)
        self.steer_state_pub = self.create_publisher(Float32MultiArray, '/debug/steer_module_states', 10)

        self.create_timer(1.0 / self.loop_hz, self.step)
        self.get_logger().info(f'Shared-control filter ready. raw={self.human_cmd_topic}, safe={self.safe_cmd_topic}')

    def raw_cb(self, msg):
        self.raw = msg

    def odom_cb(self, msg):
        self.odom = msg

    def target_cb(self, msg):
        self.target = msg

    def grid_cb(self, msg):
        self.grid = msg

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def limit_twist(self, vx, vy, wz):
        return (
            self.clamp(vx, -self.max_vx, self.max_vx),
            self.clamp(vy, -self.max_vy, self.max_vy),
            self.clamp(wz, -self.max_wz, self.max_wz),
        )

    def candidate_velocities(self, uh):
        # Sample mostly around human intent, but keep forward progress candidates.
        candidates = []
        raw_vx, raw_vy, raw_wz = uh
        base_vxs = sorted(set(self.vx_samples + [raw_vx, max(0.0, raw_vx)]))
        for vx in base_vxs:
            for dvy in self.vy_offsets:
                for dwz in self.wz_offsets:
                    candidates.append(self.limit_twist(vx, raw_vy + dvy, raw_wz + dwz))
        # include zero for emergency fallback
        candidates.append((0.0, 0.0, 0.0))
        return candidates

    def rollout(self, u):
        vx, vy, wz = u
        x = 0.0
        y = 0.0
        th = 0.0
        pts = []
        n = max(1, int(self.horizon_s / self.dt))
        for _ in range(n):
            # local body rollout approximation
            c = math.cos(th)
            s = math.sin(th)
            x += (c * vx - s * vy) * self.dt
            y += (s * vx + c * vy) * self.dt
            th += wz * self.dt
            pts.append((x, y, th))
        return pts

    def target_in_base(self):
        if self.odom is None or self.target is None:
            return None
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        yaw = yaw_from_quat(self.odom.pose.pose.orientation)
        tx = self.target.pose.position.x
        ty = self.target.pose.position.y
        dx = tx - px
        dy = ty - py
        c = math.cos(-yaw)
        s = math.sin(-yaw)
        return (c * dx - s * dy, s * dx + c * dy)

    def grid_risk(self, x, y):
        if self.grid is None:
            return 0.0
        info = self.grid.info
        ix = int((x - info.origin.position.x) / info.resolution)
        iy = int((y - info.origin.position.y) / info.resolution)
        if ix < 0 or iy < 0 or ix >= info.width or iy >= info.height:
            return 80.0
        idx = iy * info.width + ix
        val = self.grid.data[idx]
        return float(max(0, val))

    def footprint_risk(self, x, y, th):
        # sample center + four corners in local rollout frame
        L = self.robot_length * 0.5 + self.footprint_margin
        W = self.robot_width * 0.5 + self.footprint_margin
        samples = [(0.0, 0.0), (L, W), (L, -W), (-L, W), (-L, -W)]
        c = math.cos(th)
        s = math.sin(th)
        risk = 0.0
        for sx, sy in samples:
            px = x + c * sx - s * sy
            py = y + s * sx + c * sy
            risk = max(risk, self.grid_risk(px, py))
        return risk

    def score(self, u, uh):
        pts = self.rollout(u)
        max_risk = 0.0
        sum_risk = 0.0
        for x, y, th in pts:
            r = self.footprint_risk(x, y, th)
            max_risk = max(max_risk, r)
            sum_risk += r
        rejected = max_risk >= self.risk_reject_threshold

        intent = (u[0] - uh[0]) ** 2 + (u[1] - uh[1]) ** 2 + 0.35 * (u[2] - uh[2]) ** 2
        smooth = (u[0] - self.last_safe.linear.x) ** 2 + (u[1] - self.last_safe.linear.y) ** 2 + 0.25 * (u[2] - self.last_safe.angular.z) ** 2

        goal_cost = 0.0
        progress_cost = 0.0
        g = self.target_in_base()
        if g is not None and pts:
            gx, gy = g
            ex, ey, _ = pts[-1]
            # distance to semantic goal in local frame
            goal_cost = math.hypot(gx - ex, gy - ey)
            # reward progress along the line to goal
            glen = max(1e-3, math.hypot(gx, gy))
            progress = (ex * gx + ey * gy) / glen
            progress_cost = -progress

        cost = (
            self.w_intent * intent
            + self.w_risk * (sum_risk / max(1, len(pts)) / 100.0)
            + self.w_goal * goal_cost
            + self.w_progress * progress_cost
            + self.w_smooth * smooth
        )
        return cost, rejected, pts, max_risk

    def make_twist(self, u):
        msg = Twist()
        msg.linear.x = float(u[0])
        msg.linear.y = float(u[1])
        msg.angular.z = float(u[2])
        return msg

    def step(self):
        uh = self.limit_twist(self.raw.linear.x, self.raw.linear.y, self.raw.angular.z)
        best = (0.0, 0.0, 0.0)
        best_cost = 1e9
        best_pts = []
        rejected_infos = []
        accepted_infos = []

        for u in self.candidate_velocities(uh):
            cost, rejected, pts, max_risk = self.score(u, uh)
            info = (u, pts, max_risk)
            if rejected:
                rejected_infos.append(info)
                continue
            accepted_infos.append(info)
            if cost < best_cost:
                best_cost = cost
                best = u
                best_pts = pts

        # If all rejected, stop rather than driving into risk.
        if not best_pts and accepted_infos:
            best, best_pts, _ = accepted_infos[0]
        safe = self.make_twist(best)
        self.safe_pub.publish(safe)
        self.last_safe = safe

        raw_norm = math.sqrt(uh[0]**2 + uh[1]**2 + 0.25 * uh[2]**2)
        diff_norm = math.sqrt((best[0]-uh[0])**2 + (best[1]-uh[1])**2 + 0.25 * (best[2]-uh[2])**2)
        score = Float32()
        score.data = float(diff_norm / (raw_norm + 1e-3))
        self.intervention_pub.publish(score)

        self.publish_steer_modules(best)
        if self.publish_debug_markers:
            self.publish_markers(uh, best, best_pts, accepted_infos, rejected_infos)

    def line_marker(self, ns, mid, pts, rgba, width=0.035, dashed=False):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = int(mid)
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = float(width)
        m.color.r = float(rgba[0])
        m.color.g = float(rgba[1])
        m.color.b = float(rgba[2])
        m.color.a = float(rgba[3])
        for i, (x, y, th) in enumerate(pts):
            if dashed and (i % 2 == 1):
                continue
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.05
            m.points.append(p)
        return m

    def arrow_marker(self, ns, mid, u, rgba):
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = int(mid)
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x = 0.06
        m.scale.y = 0.12
        m.scale.z = 0.12
        m.color.r = float(rgba[0])
        m.color.g = float(rgba[1])
        m.color.b = float(rgba[2])
        m.color.a = float(rgba[3])
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.16
        p1 = Point()
        p1.x = float(u[0] * 1.2)
        p1.y = float(u[1] * 1.2)
        p1.z = 0.16
        m.points = [p0, p1]
        return m

    def publish_markers(self, uh, best, best_pts, accepted, rejected):
        arr = MarkerArray()
        # raw human prediction
        raw_pts = self.rollout(uh)
        arr.markers.append(self.line_marker('raw_human_prediction', 1, raw_pts, (1.0, 0.85, 0.05, 0.95), 0.035, dashed=True))

        # selected path
        arr.markers.append(self.line_marker('selected_safe_rollout', 2, best_pts, (0.0, 0.25, 1.0, 0.95), 0.055))

        # a subset of rejected and accepted candidates for readable RViz
        mid = 100
        for _, pts, _ in rejected[:12]:
            arr.markers.append(self.line_marker('rejected_rollouts', mid, pts, (1.0, 0.05, 0.05, 0.45), 0.018))
            mid += 1
        for _, pts, _ in accepted[:10]:
            arr.markers.append(self.line_marker('candidate_rollouts', mid, pts, (0.55, 0.55, 0.55, 0.28), 0.014))
            mid += 1

        # arrows
        arr.markers.append(self.arrow_marker('human_input_arrow', 300, uh, (1.0, 0.85, 0.05, 0.95)))
        arr.markers.append(self.arrow_marker('safe_output_arrow', 301, best, (0.0, 0.85, 0.25, 0.95)))

        # semantic target in base frame
        g = self.target_in_base()
        if g is not None:
            gx, gy = g
            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'semantic_target_local'
            m.id = 400
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(gx)
            m.pose.position.y = float(gy)
            m.pose.position.z = 0.12
            m.pose.orientation.w = 1.0
            m.scale.x = 0.22
            m.scale.y = 0.22
            m.scale.z = 0.22
            m.color.r = 0.1
            m.color.g = 1.0
            m.color.b = 0.1
            m.color.a = 0.85
            arr.markers.append(m)

        self.marker_pub.publish(arr)

    def publish_steer_modules(self, u):
        vx, vy, wz = u
        positions = [
            (+self.wheelbase/2.0, +self.track/2.0),
            (+self.wheelbase/2.0, -self.track/2.0),
            (-self.wheelbase/2.0, +self.track/2.0),
            (-self.wheelbase/2.0, -self.track/2.0),
        ]
        arr = MarkerArray()
        states = Float32MultiArray()
        data = []
        for i, (xw, yw) in enumerate(positions):
            # local wheel velocity caused by body twist
            wx = vx - wz * yw
            wy = vy + wz * xw
            angle = math.atan2(wy, wx) if abs(wx) + abs(wy) > 1e-4 else 0.0
            speed = math.hypot(wx, wy)
            data.extend([float(angle), float(speed)])

            m = Marker()
            m.header.frame_id = 'base_link'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'rangermini2_steer_modules'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = float(xw)
            m.pose.position.y = float(yw)
            m.pose.position.z = 0.10
            m.pose.orientation.z = math.sin(angle * 0.5)
            m.pose.orientation.w = math.cos(angle * 0.5)
            m.scale.x = 0.22
            m.scale.y = 0.045
            m.scale.z = 0.045
            m.color.r = 0.0
            m.color.g = 0.35
            m.color.b = 1.0
            m.color.a = 0.95
            arr.markers.append(m)
        states.data = data
        self.steer_marker_pub.publish(arr)
        self.steer_state_pub.publish(states)


def main():
    rclpy.init()
    node = SharedControlFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
