#!/usr/bin/env python3
"""
Ground truth people publisher for Gazebo simulation experiments.

Reads scenario definitions from config/people_scenarios.yaml and
publishes ground truth poses, markers, and optional odometry.

Topics:
  /sim/people_ground_truth  (PoseArray)    — one pose per person
  /sim/people_markers       (MarkerArray)  — colored cylinders per person
"""
import math
import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header

from ament_index_python.packages import get_package_share_directory


class SimulatedPeopleGroundTruthNode(Node):
    COLORS = [
        (1.0, 0.2, 0.2),  # red
        (0.2, 0.2, 1.0),  # blue
        (0.2, 0.8, 0.2),  # green
        (1.0, 0.6, 0.0),  # orange
        (1.0, 0.0, 1.0),  # magenta
    ]

    def __init__(self):
        super().__init__('simulated_people_ground_truth_node')

        self.declare_parameter('scenario', 'crossing_person')
        self.declare_parameter('scenario_config', '')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('publish_rate', 30.0)

        self.scenario_name = self.get_parameter('scenario').value
        self.world_frame = self.get_parameter('world_frame').value
        publish_rate = self.get_parameter('publish_rate').value

        # Load scenario config
        config_path = self.get_parameter('scenario_config').value
        if not config_path:
            pkg_share = get_package_share_directory('ranger_gazebo_experiments')
            config_path = os.path.join(pkg_share, 'config', 'people_scenarios.yaml')
        self.scenarios = self._load_config(config_path)

        if self.scenario_name not in self.scenarios:
            self.get_logger().error(
                f'Scenario "{self.scenario_name}" not found in config. '
                f'Available: {list(self.scenarios.keys())}')
            raise ValueError(f'Unknown scenario: {self.scenario_name}')

        self.scenario = self.scenarios[self.scenario_name]
        self.people_defs = self.scenario.get('people', [])
        self.get_logger().info(
            f'Scenario: {self.scenario_name} — '
            f'{len(self.people_defs)} people: {[p["name"] for p in self.people_defs]}')

        # Publishers
        self.gt_pub = self.create_publisher(PoseArray, '/sim/people_ground_truth', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/sim/people_markers', 10)

        # Timer
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'simulated_people_ground_truth_node started: '
            f'scenario={self.scenario_name} rate={publish_rate}Hz')

    def _load_config(self, path):
        if not os.path.isfile(path):
            self.get_logger().fatal(f'Config file not found: {path}')
            raise FileNotFoundError(path)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        return data.get('scenarios', {})

    def _interpolate_trajectory(self, person_def, elapsed):
        """Interpolate position along trajectory waypoints based on elapsed time."""
        pts = person_def.get('trajectory_points', [])
        speed = person_def.get('speed', 1.0)
        loop = person_def.get('loop', True)
        pause = person_def.get('pause_time', 0.0)

        if len(pts) < 2:
            return pts[0] if pts else [0.0, 0.0, 0.0]

        # Calculate segment distances and cumulative times
        seg_dists = []
        seg_times = []
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            seg_dists.append(math.hypot(dx, dy))
            seg_times.append(seg_dists[-1] / max(speed, 0.01))

        total_time = sum(seg_times) + pause

        if loop and total_time > 0.01:
            elapsed = elapsed % total_time
        else:
            elapsed = min(elapsed, total_time)

        # Find which segment we're in
        cum_time = 0.0
        for i, seg_t in enumerate(seg_times):
            if elapsed <= cum_time + seg_t:
                # In this segment
                seg_elapsed = elapsed - cum_time
                alpha = seg_elapsed / max(seg_t, 0.001)
                alpha = max(0.0, min(1.0, alpha))
                p0 = pts[i]
                p1 = pts[i + 1]
                return [
                    p0[0] + alpha * (p1[0] - p0[0]),
                    p0[1] + alpha * (p1[1] - p0[1]),
                    p0[2] + alpha * (p1[2] - p0[2]) if len(p0) > 2 else 0.0,
                ]
            cum_time += seg_t

        # In pause or at end
        return pts[-1] if len(pts) > 0 else [0.0, 0.0, 0.0]

    def _publish(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = now.to_msg()
        pose_array.header.frame_id = self.world_frame

        markers = MarkerArray()

        for i, person_def in enumerate(self.people_defs):
            pos = self._interpolate_trajectory(person_def, elapsed)

            # Pose for PoseArray
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2]) if len(pos) > 2 else 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

            # Marker
            color = self.COLORS[i % len(self.COLORS)]
            m = Marker()
            m.header = pose_array.header
            m.ns = 'sim_person'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2]) + 0.875 if len(pos) > 2 else 0.875
            m.pose.orientation.w = 1.0
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 1.75
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 0.8
            markers.markers.append(m)

            # Text label marker
            text_m = Marker()
            text_m.header = pose_array.header
            text_m.ns = 'sim_person_label'
            text_m.id = i + 10000
            text_m.type = Marker.TEXT_VIEW_FACING
            text_m.action = Marker.ADD
            text_m.pose.position.x = float(pos[0])
            text_m.pose.position.y = float(pos[1])
            text_m.pose.position.z = float(pos[2]) + 2.0
            text_m.scale.z = 0.3
            text_m.color.r, text_m.color.g, text_m.color.b = 1.0, 1.0, 1.0
            text_m.color.a = 0.9
            text_m.text = f'{person_def["name"]} (id={i})'
            markers.markers.append(text_m)

        self.gt_pub.publish(pose_array)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SimulatedPeopleGroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
