#!/usr/bin/env python3
"""Bridge FAST-LIO pose and wheel odometry into Nav2's map->odom TF.

FAST-LIO publishes camera_init->body for the tilted LiDAR/IMU.  Nav2 needs the
planar chassis chain map->odom->base_link.  This node removes the measured
base_link->livox_frame mounting transform, compares the result with wheel
odometry, and publishes map->odom.  It never publishes velocity commands.
"""

import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]  # x, y, z, w
Transform = Tuple[Vec3, Quat]


def q_normalize(q: Quat) -> Quat:
    n = math.sqrt(sum(v * v for v in q))
    return (0.0, 0.0, 0.0, 1.0) if n < 1e-12 else tuple(v / n for v in q)


def q_mul(a: Quat, b: Quat) -> Quat:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return q_normalize((
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ))


def q_inverse(q: Quat) -> Quat:
    x, y, z, w = q_normalize(q)
    return (-x, -y, -z, w)


def rotate(q: Quat, v: Vec3) -> Vec3:
    x, y, z, w = q_normalize(q)
    vx, vy, vz = v
    tx, ty, tz = (2.0 * (y * vz - z * vy),
                  2.0 * (z * vx - x * vz),
                  2.0 * (x * vy - y * vx))
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def compose(a: Transform, b: Transform) -> Transform:
    at, aq = a
    bt, bq = b
    rbt = rotate(aq, bt)
    return ((at[0] + rbt[0], at[1] + rbt[1], at[2] + rbt[2]), q_mul(aq, bq))


def inverse(t: Transform) -> Transform:
    xyz, q = t
    qi = q_inverse(q)
    return (rotate(qi, (-xyz[0], -xyz[1], -xyz[2])), qi)


def yaw_quaternion(yaw: float) -> Quat:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_from_quaternion(q: Quat) -> float:
    x, y, z, w = q_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_transform(msg: Odometry) -> Transform:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))


class FastlioNavTf(Node):
    def __init__(self):
        super().__init__('fastlio_nav_tf')
        self.declare_parameter('lio_topic', '/Odometry')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sensor_x', 0.30)
        self.declare_parameter('sensor_y', 0.0)
        self.declare_parameter('sensor_z', 0.70)
        self.declare_parameter('sensor_roll', 0.0)
        self.declare_parameter('sensor_pitch', 0.523599)
        self.declare_parameter('sensor_yaw', 0.0)
        self.declare_parameter('planarize', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planarize = self.get_parameter('planarize').value
        roll = self.get_parameter('sensor_roll').value
        pitch = self.get_parameter('sensor_pitch').value
        yaw = self.get_parameter('sensor_yaw').value
        cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
        cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
        cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        q = (sr * cp * cy - cr * sp * sy,
             cr * sp * cy + sr * cp * sy,
             cr * cp * sy - sr * sp * cy,
             cr * cp * cy + sr * sp * sy)
        self.base_sensor = ((self.get_parameter('sensor_x').value,
                             self.get_parameter('sensor_y').value,
                             self.get_parameter('sensor_z').value), q)
        self.wheel_odom = None
        self.tf_broadcaster = TransformBroadcaster(self)
        self.base_odom_pub = self.create_publisher(Odometry, '/lio/base_odom', 10)
        self.create_subscription(
            Odometry, self.get_parameter('wheel_odom_topic').value,
            self.wheel_callback, 20)
        self.create_subscription(
            Odometry, self.get_parameter('lio_topic').value,
            self.lio_callback, qos_profile_sensor_data)
        self.get_logger().info('Waiting for /odom and /Odometry; this node sends no cmd_vel')

    def wheel_callback(self, msg: Odometry):
        self.wheel_odom = msg

    def lio_callback(self, msg: Odometry):
        if self.wheel_odom is None:
            return
        camera_init_sensor = pose_transform(msg)
        # FAST-LIO expresses motion in camera_init, whose initial orientation
        # is the physically tilted sensor frame. Lift that pose into the
        # level initial chassis map frame before removing the sensor mount.
        map_sensor = compose(self.base_sensor, camera_init_sensor)
        odom_base = pose_transform(self.wheel_odom)
        map_base = compose(map_sensor, inverse(self.base_sensor))
        map_odom = compose(map_base, inverse(odom_base))
        if self.planarize:
            map_odom = ((map_odom[0][0], map_odom[0][1], 0.0),
                        yaw_quaternion(yaw_from_quaternion(map_odom[1])))
            map_base = ((map_base[0][0], map_base[0][1], 0.0),
                        yaw_quaternion(yaw_from_quaternion(map_base[1])))

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map_odom[0]
        q = map_odom[1]
        tf.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.tf_broadcaster.sendTransform(tf)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map_base[0]
        q = map_base[1]
        out.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        out.pose.covariance = msg.pose.covariance
        self.base_odom_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FastlioNavTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
