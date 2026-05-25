#!/usr/bin/env python3
"""
Lightweight reactive person avoidance controller for Gazebo experiments.

Subscribes to /odom, /fused_obstacles (or /obstacles_mid360, /obstacles_d435i,
/obstacles_yolo_person depending on mode), and optionally /sim/people_ground_truth.

Publishes /cmd_vel, /local_goal, /avoidance_debug.

Strategy:
  1. Direct approach to goal when no obstacles nearby.
  2. Slow down when person/dynamic obstacle within slow_down_distance.
  3. Stop when within stop_distance.
  4. Lateral avoidance when blocked and side clearance available.
  5. Goal reached → stop + publish success event.

This is a baseline controller for ablation studies, not a replacement for Nav2.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Bool

from .utils_geometry import quaternion_to_euler, euler_to_quaternion
from .utils_metrics import compute_min_distance_to_person


class PersonAvoidanceController(Node):
    def __init__(self):
        super().__init__('person_avoidance_controller')

        # Parameters
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('slow_down_distance', 2.0)
        self.declare_parameter('stop_distance', 0.8)
        self.declare_parameter('safety_radius', 0.5)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('goal_x', 8.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('use_fused_obstacles', True)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('use_yolo', True)
        self.declare_parameter('use_ground_truth_for_control', False)
        self.declare_parameter('fused_obstacles_topic', '/fused_obstacles')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('ground_truth_topic', '/sim/people_ground_truth')
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('frame_id', 'odom')

        p = lambda n: self.get_parameter(n).value
        self.max_v = p('max_linear_speed')
        self.max_w = p('max_angular_speed')
        self.slow_dist = p('slow_down_distance')
        self.stop_dist = p('stop_distance')
        self.safety_r = p('safety_radius')
        self.goal_tol = p('goal_tolerance')
        self.goal_x = p('goal_x')
        self.goal_y = p('goal_y')
        self.goal_yaw = p('goal_yaw')
        self.use_fused = p('use_fused_obstacles')
        self.use_lidar = p('use_lidar')
        self.use_depth = p('use_depth')
        self.use_yolo = p('use_yolo')
        self.use_gt_ctrl = p('use_ground_truth_for_control')
        self.frame_id = p('frame_id')

        # State
        self.robot_pose = (0.0, 0.0, 0.0)
        self.robot_vel = (0.0, 0.0)
        self.fused_obstacles = []
        self.gt_people = []
        self.goal_reached = False
        self.stopped = False
        self.stop_count = 0
        self.collision_count = 0
        self.dangerous_close_count = 0
        self.prev_obstacle_dist = float('inf')

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, p('odom_topic'), self._odom_cb, 10)
        self.fused_sub = self.create_subscription(
            MarkerArray, p('fused_obstacles_topic'), self._fused_cb, 10)
        self.gt_sub = self.create_subscription(
            PoseArray, p('ground_truth_topic'), self._gt_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_goal_pub = self.create_publisher(
            PoseStamped, '/local_goal', 10)
        self.risk_pub = self.create_publisher(
            MarkerArray, '/risk_markers', 10)
        self.debug_pub = self.create_publisher(
            String, '/avoidance_debug', 10)
        self.success_pub = self.create_publisher(
            Bool, '/experiment_success', 10)

        # Timer
        dt = 1.0 / p('control_rate')
        self.timer = self.create_timer(dt, self._control_loop)
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'person_avoidance_controller started: goal=({self.goal_x},{self.goal_y}) '
            f'slow={self.slow_dist}m stop={self.stop_dist}m')

    def _odom_cb(self, msg):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.robot_pose = (px, py, yaw)
        self.robot_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    def _fused_cb(self, msg):
        self.fused_obstacles = []
        for m in msg.markers:
            self.fused_obstacles.append((m.pose.position.x, m.pose.position.y))

    def _gt_cb(self, msg):
        self.gt_people = [(p.position.x, p.position.y) for p in msg.poses]

    def _get_obstacle_positions(self):
        """Return list of (x, y) obstacle positions based on enabled sources."""
        obstacles = []
        if self.use_fused:
            obstacles.extend(self.fused_obstacles)
        if self.use_gt_ctrl:
            obstacles.extend(self.gt_people)
        return obstacles

    def _control_loop(self):
        if self.goal_reached:
            self._publish_cmd_vel(0.0, 0.0)
            return

        rx, ry, ryaw = self.robot_pose

        # Check goal
        dist_to_goal = math.hypot(self.goal_x - rx, self.goal_y - ry)
        if dist_to_goal < self.goal_tol:
            self.goal_reached = True
            self._publish_cmd_vel(0.0, 0.0)
            self.get_logger().info(f'Goal reached! dist={dist_to_goal:.3f}m')
            msg = Bool()
            msg.data = True
            self.success_pub.publish(msg)
            return

        # Get obstacles
        obstacles = self._get_obstacle_positions()

        # Compute distances
        min_dist = float('inf')
        closest_obs = None
        for obs in obstacles:
            d = math.hypot(rx - obs[0], ry - obs[1])
            if d < min_dist:
                min_dist = d
                closest_obs = obs

        # Collision / dangerous close check
        if min_dist < 0.25:
            self.collision_count += 1
        if min_dist < self.safety_r:
            self.dangerous_close_count += 1

        # Heading to goal
        goal_angle = math.atan2(self.goal_y - ry, self.goal_x - rx)
        angle_error = goal_angle - ryaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Control logic
        if closest_obs and min_dist < self.stop_dist:
            # Too close — stop
            self._publish_cmd_vel(0.0, 0.0)
            self.stopped = True
            self.stop_count += 1
            self._publish_debug(f'STOP: obstacle at {min_dist:.2f}m')

        elif closest_obs and min_dist < self.slow_dist:
            # Slow down and try lateral avoidance
            speed = self.max_v * (min_dist - self.stop_dist) / (self.slow_dist - self.stop_dist)
            speed = max(0.05, speed)

            # Check side clearance
            obs_angle = math.atan2(closest_obs[1] - ry, closest_obs[0] - rx)
            side = 1.0 if math.sin(obs_angle - ryaw) > 0 else -1.0

            # Blend: move toward goal but bias away from obstacle
            if abs(angle_error) > 0.3:
                # Facing away from goal — turn first
                w = self.max_w * (angle_error / math.pi)
                self._publish_cmd_vel(0.0, w)
            else:
                # Move forward with lateral bias
                w = side * 0.3 * self.max_w * (1.0 - min_dist / self.slow_dist)
                self._publish_cmd_vel(speed, w)
            self.stopped = False
            self._publish_debug(f'SLOW: {min_dist:.2f}m speed={speed:.2f}')

        else:
            # No obstacle — direct to goal
            speed = min(self.max_v, dist_to_goal * 0.5)
            w = self.max_w * (angle_error / math.pi)
            w = max(-self.max_w, min(self.max_w, w))
            self._publish_cmd_vel(speed, w)
            self.stopped = False
            self._publish_debug(f'GO: dist={dist_to_goal:.2f}m')

        # Publish local goal
        lg = PoseStamped()
        lg.header.frame_id = self.frame_id
        lg.header.stamp = self.get_clock().now().to_msg()
        lg.pose.position.x = self.goal_x
        lg.pose.position.y = self.goal_y
        lg.pose.orientation.w = 1.0
        self.local_goal_pub.publish(lg)

        # Publish risk markers
        self._publish_risk_markers(obstacles, min_dist)

        self.prev_obstacle_dist = min_dist

    def _publish_cmd_vel(self, vx, wz):
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def _publish_debug(self, text):
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)

    def _publish_risk_markers(self, obstacles, min_dist):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, obs in enumerate(obstacles):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = 'avoidance_risk'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = obs[0]
            m.pose.position.y = obs[1]
            m.pose.position.z = 0.5
            m.scale.x = m.scale.y = m.scale.z = self.safety_r * 2.0
            d = math.hypot(self.robot_pose[0] - obs[0], self.robot_pose[1] - obs[1])
            if d < self.stop_dist:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            elif d < self.slow_dist:
                m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0
            m.color.a = 0.3
            markers.markers.append(m)
        self.risk_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = PersonAvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
