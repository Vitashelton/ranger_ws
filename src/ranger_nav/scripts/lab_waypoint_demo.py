#!/usr/bin/env python3
"""Record real lab poses and navigate to rooms/elevator by name."""

import argparse
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

import yaml


DEFAULT_DB = Path(
    os.environ.get(
        'RANGER_LAB_WAYPOINTS',
        '~/.config/ranger_nav/lab_waypoints.yaml',
    )
).expanduser()


def empty_database(map_id=None):
    return {
        'map_id': map_id,
        'frame_id': 'map',
        'waypoints': {},
        'routes': {},
        'aliases': {},
    }


def load_database(path):
    path = Path(path).expanduser()
    if not path.exists():
        return empty_database()
    with path.open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML mapping')
    result = empty_database()
    result.update(data)
    for key in ('waypoints', 'routes', 'aliases'):
        if not isinstance(result.get(key), dict):
            raise ValueError(f'{key} must be a mapping')
    return result


def save_database(path, data):
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary = tempfile.mkstemp(
        prefix=path.name + '.', suffix='.tmp', dir=str(path.parent)
    )
    try:
        with os.fdopen(handle, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=False)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def load_map_id(path):
    path = Path(path).expanduser()
    if not path.exists():
        raise RuntimeError(f'map identity file does not exist: {path}')
    map_id = path.read_text(encoding='utf-8').strip()
    if not map_id:
        raise RuntimeError(f'map identity file is empty: {path}')
    return map_id


def require_matching_map(args, data):
    current = load_map_id(args.map_id_file)
    stored = data.get('map_id')
    if stored != current:
        raise RuntimeError(
            'waypoints do not belong to the loaded map '
            f'(database={stored!r}, current={current!r}); '
            'run real_lab.sh reset after saving the intended map'
        )
    return current


def resolve_target(data, target):
    canonical = data['aliases'].get(target, target)
    if canonical in data['routes']:
        names = list(data['routes'][canonical])
        if not names:
            raise ValueError(f'route {canonical!r} is empty')
    elif canonical in data['waypoints']:
        names = [canonical]
    else:
        raise ValueError(f'unknown waypoint or route: {target!r}')
    missing = [name for name in names if name not in data['waypoints']]
    if missing:
        raise ValueError(f'route {canonical!r} has missing waypoints: {missing}')
    return canonical, names


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def quaternion_from_yaw(yaw):
    from geometry_msgs.msg import Quaternion
    return Quaternion(
        x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0)
    )


def record_waypoint(args, data):
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    require_matching_map(args, data)

    class PoseRecorder(Node):
        def __init__(self):
            super().__init__('lab_waypoint_recorder')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)

    rclpy.init()
    node = PoseRecorder()
    deadline = time.monotonic() + args.timeout
    try:
        transform = None
        while rclpy.ok() and transform is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                transform = node.buffer.lookup_transform(
                    args.frame_id, args.base_frame, rclpy.time.Time())
            except Exception:
                pass
        if transform is None:
            raise RuntimeError(
                f'no TF {args.frame_id}->{args.base_frame} '
                f'in {args.timeout:.1f}s'
            )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        frame_id = args.frame_id
        if data['waypoints'] and frame_id != data.get('frame_id'):
            raise RuntimeError(
                f'pose frame changed from {data.get("frame_id")} to {frame_id}'
            )
        data['frame_id'] = frame_id
        data['waypoints'][args.name] = {
            'x': round(float(translation.x), 4),
            'y': round(float(translation.y), 4),
            'yaw': round(float(yaw_from_quaternion(rotation)), 6),
        }
        save_database(args.file, data)
        point = data['waypoints'][args.name]
        print(
            f"recorded {args.name}: frame={frame_id} "
            f"x={point['x']:.3f} y={point['y']:.3f} yaw={point['yaw']:.3f}"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def navigate(args, data):
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
    from rclpy.action import ActionClient
    from rclpy.node import Node

    require_matching_map(args, data)
    canonical, names = resolve_target(data, args.target)
    print(f'{canonical}: ' + ' -> '.join(names))
    if args.dry_run:
        return

    class NamedNavigator(Node):
        def __init__(self):
            super().__init__('lab_named_navigator')
            self.client = ActionClient(self, NavigateToPose, args.action_name)

        def run_goal(self, name):
            if not self.client.wait_for_server(timeout_sec=args.server_timeout):
                raise RuntimeError(f'action server {args.action_name} is unavailable')
            point = data['waypoints'][name]
            pose = PoseStamped()
            pose.header.frame_id = data.get('frame_id', 'map')
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point['x'])
            pose.pose.position.y = float(point['y'])
            pose.pose.orientation = quaternion_from_yaw(float(point['yaw']))
            goal = NavigateToPose.Goal()
            goal.pose = pose
            future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)
            handle = future.result()
            if handle is None or not handle.accepted:
                return False, 'goal rejected'
            print(f'navigating to {name} ...')
            result_future = handle.get_result_async()
            deadline = None if args.goal_timeout <= 0.0 else (
                time.monotonic() + args.goal_timeout
            )
            try:
                while rclpy.ok() and not result_future.done():
                    rclpy.spin_once(self, timeout_sec=0.2)
                    if deadline is not None and time.monotonic() >= deadline:
                        cancel = handle.cancel_goal_async()
                        rclpy.spin_until_future_complete(self, cancel)
                        return False, 'goal timeout; cancel requested'
            except KeyboardInterrupt:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel)
                return False, 'interrupted; cancel requested'
            wrapped = result_future.result()
            if wrapped.status == GoalStatus.STATUS_SUCCEEDED:
                return True, 'succeeded'
            return False, f'finished with action status {wrapped.status}'

    rclpy.init()
    node = NamedNavigator()
    try:
        for name in names:
            succeeded, detail = node.run_goal(name)
            print(f'{name}: {detail}')
            if not succeeded:
                raise RuntimeError(f'route stopped at {name}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def print_database(data):
    print(f"map_id: {data.get('map_id')}")
    print(f"frame_id: {data.get('frame_id', 'map')}")
    print('waypoints:')
    for name, point in data['waypoints'].items():
        print(
            f"  {name}: x={float(point['x']):.3f} "
            f"y={float(point['y']):.3f} yaw={float(point['yaw']):.3f}"
        )
    print('routes:')
    for name, route in data['routes'].items():
        print(f"  {name}: {' -> '.join(route) if route else '(empty)'}")
    print('aliases:')
    for alias, target in data['aliases'].items():
        print(f'  {alias} -> {target}')


def publish_markers(args):
    import rclpy
    from geometry_msgs.msg import Point
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from visualization_msgs.msg import Marker, MarkerArray

    class MarkerPublisher(Node):
        def __init__(self):
            super().__init__('lab_waypoint_markers')
            qos = QoSProfile(depth=1)
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            qos.reliability = ReliabilityPolicy.RELIABLE
            self.publisher = self.create_publisher(
                MarkerArray, '/lab_waypoints/markers', qos
            )
            self.create_timer(1.0, self.publish_database)
            self.publish_database()

        @staticmethod
        def color(marker, red, green, blue, alpha=1.0):
            marker.color.r = red
            marker.color.g = green
            marker.color.b = blue
            marker.color.a = alpha

        def publish_database(self):
            try:
                data = load_database(args.file)
            except Exception as error:
                self.get_logger().error(f'cannot load {args.file}: {error}')
                return
            array = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            array.markers.append(clear)
            frame_id = data.get('frame_id', 'map')
            marker_id = 1
            for name, waypoint in data['waypoints'].items():
                arrow = Marker()
                arrow.header.frame_id = frame_id
                arrow.header.stamp = self.get_clock().now().to_msg()
                arrow.ns = 'lab_waypoints'
                arrow.id = marker_id
                marker_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position.x = float(waypoint['x'])
                arrow.pose.position.y = float(waypoint['y'])
                arrow.pose.position.z = 0.10
                arrow.pose.orientation = quaternion_from_yaw(float(waypoint['yaw']))
                arrow.scale.x = 0.65
                arrow.scale.y = 0.14
                arrow.scale.z = 0.14
                self.color(arrow, 0.10, 0.85, 0.25)
                array.markers.append(arrow)

                label = Marker()
                label.header = arrow.header
                label.ns = 'lab_waypoint_labels'
                label.id = marker_id
                marker_id += 1
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = float(waypoint['x'])
                label.pose.position.y = float(waypoint['y'])
                label.pose.position.z = 0.65
                label.pose.orientation.w = 1.0
                label.scale.z = 0.32
                label.text = name
                self.color(label, 1.0, 1.0, 1.0)
                array.markers.append(label)

            for route_name, route in data['routes'].items():
                points = [
                    data['waypoints'][name]
                    for name in route
                    if name in data['waypoints']
                ]
                if len(points) < 2:
                    continue
                line = Marker()
                line.header.frame_id = frame_id
                line.header.stamp = self.get_clock().now().to_msg()
                line.ns = 'lab_routes'
                line.id = marker_id
                marker_id += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.07
                line.text = route_name
                self.color(line, 0.15, 0.55, 1.0, 0.85)
                for waypoint in points:
                    line.points.append(Point(
                        x=float(waypoint['x']),
                        y=float(waypoint['y']),
                        z=0.04,
                    ))
                array.markers.append(line)
            self.publisher.publish(array)

    rclpy.init()
    node = MarkerPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser():
    parser = argparse.ArgumentParser(
        description='Record and navigate real laboratory waypoints.'
    )
    parser.add_argument('--file', type=Path, default=DEFAULT_DB)
    parser.add_argument(
        '--map-id-file', type=Path,
        default=Path('~/.config/ranger_nav/maps/real_lab.map_id').expanduser(),
    )
    commands = parser.add_subparsers(dest='command', required=True)

    record = commands.add_parser('record', help='record current global map pose')
    record.add_argument('name')
    record.add_argument('--frame-id', default='map')
    record.add_argument('--base-frame', default='base_link')
    record.add_argument('--timeout', type=float, default=5.0)

    route = commands.add_parser('route', help='define an ordered named route')
    route.add_argument('name')
    route.add_argument('waypoints', nargs='+')

    alias = commands.add_parser('alias', help='add aliases for a target')
    alias.add_argument('target')
    alias.add_argument('aliases', nargs='+')

    go = commands.add_parser('go', help='navigate to a waypoint or route')
    go.add_argument('target')
    go.add_argument('--action-name', default='/navigate_to_pose')
    go.add_argument('--server-timeout', type=float, default=10.0)
    go.add_argument('--goal-timeout', type=float, default=0.0)
    go.add_argument('--dry-run', action='store_true')

    commands.add_parser('list', help='show recorded waypoints and routes')
    commands.add_parser('markers', help='publish waypoint markers for PC RViz')
    commands.add_parser('reset', help='archive waypoints and bind to current map')
    return parser


def main():
    from rclpy.utilities import remove_ros_args

    args = build_parser().parse_args(remove_ros_args(args=sys.argv)[1:])
    data = load_database(args.file)
    if args.command == 'record':
        record_waypoint(args, data)
    elif args.command == 'route':
        missing = [name for name in args.waypoints if name not in data['waypoints']]
        if missing:
            raise SystemExit(f'cannot create route; missing waypoints: {missing}')
        data['routes'][args.name] = args.waypoints
        save_database(args.file, data)
        print(f"saved route {args.name}: {' -> '.join(args.waypoints)}")
    elif args.command == 'alias':
        if args.target not in data['waypoints'] and args.target not in data['routes']:
            raise SystemExit(f'unknown alias target: {args.target}')
        for alias_name in args.aliases:
            data['aliases'][alias_name] = args.target
        save_database(args.file, data)
        print(f"saved aliases for {args.target}: {', '.join(args.aliases)}")
    elif args.command == 'go':
        navigate(args, data)
    elif args.command == 'list':
        print_database(data)
    elif args.command == 'markers':
        require_matching_map(args, data)
        publish_markers(args)
    elif args.command == 'reset':
        map_id = load_map_id(args.map_id_file)
        path = Path(args.file).expanduser()
        if path.exists():
            archive = path.with_name(
                f'{path.stem}.archive-{time.strftime("%Y%m%d-%H%M%S")}{path.suffix}'
            )
            archive.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, archive)
            print(f'archived old waypoints: {archive}')
        save_database(path, empty_database(map_id))
        print(f'waypoint database now belongs to map {map_id}')


if __name__ == '__main__':
    main()
