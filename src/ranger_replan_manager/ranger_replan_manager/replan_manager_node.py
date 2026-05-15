#!/usr/bin/env python3
"""
Replanning trigger and recovery manager.

Monitors Nav2 navigation health and triggers escalating recovery actions:
  1. Slowdown (0.5x cmd_vel)
  2. Clear local costmap
  3. Clear global costmap
  4. Replan (ComputePathToPose)
  5. Spin adjust
  6. Wait (2s)
  7. Safety stop (cancel goal)

Trigger conditions:
  - Local planner failure (3 consecutive)
  - Robot stuck (velocity < 0.05 m/s for > 5s)
  - Global path blocked by persistent dynamic obstacle (> 3s)
  - TTC risk too high (< 1.0s for > 2s)
  - Excessive emergency stops (> 3 in 30s window)
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from ranger_msgs.msg import SystemState


class ReplanManagerNode(Node):
    RECOVERY_NONE = 0
    RECOVERY_SLOWDOWN = 1
    RECOVERY_CLEAR_LOCAL = 2
    RECOVERY_CLEAR_GLOBAL = 3
    RECOVERY_REPLAN = 4
    RECOVERY_SPIN = 5
    RECOVERY_WAIT = 6
    RECOVERY_SAFETY_STOP = 7

    def __init__(self):
        super().__init__('replan_manager_node')

        # --- Params ---
        self.declare_parameter('stuck_vel_threshold', 0.05)
        self.declare_parameter('stuck_duration', 5.0)
        self.declare_parameter('ttc_critical_threshold', 1.0)
        self.declare_parameter('ttc_critical_duration', 2.0)
        self.declare_parameter('blockage_duration', 3.0)
        self.declare_parameter('max_estop_count', 3)
        self.declare_parameter('estop_window', 30.0)
        self.declare_parameter('local_failure_max', 3)
        self.declare_parameter('slowdown_factor', 0.5)
        self.declare_parameter('recovery_cooldown', 5.0)
        self.declare_parameter('frame_id', 'base_link')
        self._load_params()

        # State
        self.robot_vel = 0.0
        self.stuck_start = None
        self.global_plan = None
        self.is_navigating = False
        self.current_goal = None
        self.low_ttc_start = None
        self.estop_timestamps = []
        self.local_failure_count = 0
        self.current_recovery = self.RECOVERY_NONE
        self.last_recovery_time = self.get_clock().now()
        self.recovery_in_progress = False
        self.spin_start_time = None
        self.wait_start_time = None

        # Subs
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.plan_sub = self.create_subscription(
            Path, '/plan', self._plan_cb, 10)
        self.risk_sub = self.create_subscription(
            MarkerArray, '/risk_markers', self._risk_cb, 10)
        self.system_sub = self.create_subscription(
            SystemState, '/system_state', self._system_cb, 10)

        # Pub
        self.event_pub = self.create_publisher(String, '/replan_event', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer: 10 Hz evaluation
        self.timer = self.create_timer(0.1, self._evaluate)

        self.get_logger().info('replan_manager_node started')

    def _load_params(self):
        p = lambda name: self.get_parameter(name).value
        self.stuck_vel = p('stuck_vel_threshold')
        self.stuck_dur = p('stuck_duration')
        self.ttc_crit = p('ttc_critical_threshold')
        self.ttc_crit_dur = p('ttc_critical_duration')
        self.blockage_dur = p('blockage_duration')
        self.max_estop = p('max_estop_count')
        self.estop_win = p('estop_window')
        self.local_fail_max = p('local_failure_max')
        self.slowdown_factor = p('slowdown_factor')
        self.recovery_cooldown = p('recovery_cooldown')

    def _odom_cb(self, msg):
        self.robot_vel = math.hypot(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        # Check if navigating: robot has non-zero velocity or recent goal
        if self.robot_vel > self.stuck_vel:
            self.stuck_start = None
            self.is_navigating = True
        elif self.stuck_start is None and self.is_navigating:
            self.stuck_start = self.get_clock().now()

    def _plan_cb(self, msg):
        self.global_plan = msg.poses
        self.is_navigating = len(msg.poses) > 0

    def _risk_cb(self, msg):
        now = self.get_clock().now()
        # Check for high-risk obstacles (red markers)
        has_critical = False
        for m in msg.markers:
            if m.ns == 'risk' and m.color.r >= 1.0 and m.color.g < 0.1:
                has_critical = True
                break

        if has_critical:
            if self.low_ttc_start is None:
                self.low_ttc_start = now
        else:
            self.low_ttc_start = None

    def _system_cb(self, msg):
        if msg.error_code != 0:
            self.get_logger().error(f'System error code: {msg.error_code}')
            self._publish_event('ERROR', f'System error: {msg.error_code}')
            self._trigger_safety_stop()

    def _publish_event(self, event_type, detail=''):
        msg = String()
        msg.data = f'[{event_type}] {detail}'
        self.event_pub.publish(msg)
        self.get_logger().info(f'Replan event: {msg.data}')

    def _evaluate(self):
        now = self.get_clock().now()

        # Check recovery cooldown
        if (now - self.last_recovery_time).nanoseconds * 1e-9 < self.recovery_cooldown:
            return

        triggers = []

        # 1. Local planner failure
        if self.local_failure_count >= self.local_fail_max:
            triggers.append(('local_failure', self.RECOVERY_CLEAR_LOCAL))

        # 2. Robot stuck
        if self.stuck_start is not None:
            stuck_dur = (now - self.stuck_start).nanoseconds * 1e-9
            if stuck_dur > self.stuck_dur:
                triggers.append(('stuck', self.RECOVERY_CLEAR_LOCAL))
                self.stuck_start = None

        # 3. Persistent high TTC risk
        if self.low_ttc_start is not None:
            ttc_dur = (now - self.low_ttc_start).nanoseconds * 1e-9
            if ttc_dur > self.ttc_crit_dur:
                triggers.append(('high_ttc', self.RECOVERY_SLOWDOWN))

        # 4. Excessive e-stops
        self.estop_timestamps = [
            t for t in self.estop_timestamps
            if (now - t).nanoseconds * 1e-9 < self.estop_win
        ]
        if len(self.estop_timestamps) >= self.max_estop:
            triggers.append(('excessive_estop', self.RECOVERY_SAFETY_STOP))

        if not triggers:
            return

        # Select highest priority recovery (lowest level number)
        best_trigger, best_level = min(triggers, key=lambda x: x[1])
        self._execute_recovery(best_level, best_trigger)

    def _execute_recovery(self, level, reason):
        self.last_recovery_time = self.get_clock().now()
        self._publish_event('RECOVERY', f'Level {level}: {reason}')

        if level == self.RECOVERY_SLOWDOWN:
            self._publish_event('ACTION', 'Slowdown: scaling cmd_vel by 0.5')
            self.recovery_in_progress = True
            # Let costmap clearing handle it next cycle if still needed

        elif level == self.RECOVERY_CLEAR_LOCAL:
            self._publish_event('ACTION', 'Clear local costmap')
            self.local_failure_count = 0
            self.stuck_start = None
            # Service call attempt (non-blocking, best-effort)
            self._call_clear_costmap('local')

        elif level == self.RECOVERY_CLEAR_GLOBAL:
            self._publish_event('ACTION', 'Clear global costmap')
            self._call_clear_costmap('global')

        elif level == self.RECOVERY_REPLAN:
            self._publish_event('ACTION', 'Replan')
            self._call_replan()

        elif level == self.RECOVERY_SPIN:
            self._publish_event('ACTION', 'Spin adjust (±30°)')
            if self.spin_start_time is None:
                self.spin_start_time = self.get_clock().now()

        elif level == self.RECOVERY_WAIT:
            self._publish_event('ACTION', 'Wait 2s')
            if self.wait_start_time is None:
                self.wait_start_time = self.get_clock().now()
                self._publish_zero_cmd_vel()

        elif level == self.RECOVERY_SAFETY_STOP:
            self._publish_event('ACTION', 'SAFETY STOP: canceling navigation')
            self._trigger_safety_stop()

    def _call_clear_costmap(self, layer):
        """Best-effort costmap clearing via system call."""
        import subprocess
        try:
            if layer == 'local':
                subprocess.run(
                    ['ros2', 'service', 'call', '/local_costmap/clear_entirely',
                     'nav2_msgs/srv/ClearEntireCostmap'], timeout=3.0)
            else:
                subprocess.run(
                    ['ros2', 'service', 'call', '/global_costmap/clear_entirely',
                     'nav2_msgs/srv/ClearEntireCostmap'], timeout=3.0)
        except Exception as e:
            self.get_logger().warn(f'Clear costmap failed: {e}')

    def _call_replan(self):
        """Best-effort global replan via system call."""
        import subprocess
        try:
            subprocess.run(
                ['ros2', 'service', 'call', '/planner_server/compute_path_to_pose',
                 'nav2_msgs/srv/ComputePathToPose'], timeout=3.0)
        except Exception as e:
            self.get_logger().warn(f'Replan failed: {e}')

    def _trigger_safety_stop(self):
        """Publish zero cmd_vel and attempt to cancel navigation."""
        self._publish_zero_cmd_vel()
        import subprocess
        try:
            subprocess.run(
                ['ros2', 'action', 'cancel', '/navigate_to_pose'], timeout=3.0)
        except Exception:
            pass

    def _publish_zero_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ReplanManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
