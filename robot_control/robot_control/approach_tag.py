"""Explicitly started short approach to a tag; publish wheel commands at 20 Hz."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control.approach_policy import ApproachPolicy
from robot_control.square_approach import SquareApproachPolicy
from robot_control.tag_distance import CAMERA_OFFSET_FROM_ROBOT_CENTER_M


class ApproachTag(Node):
    def __init__(self, node_name='approach_tag', defaults=None):
        super().__init__(node_name)
        defaults = defaults or {}
        for name, default in [('tag_frame', 'tag_0'), ('target_distance', 1.0),
                              ('max_command', 0.08), ('max_duration', 20.0),
                              ('min_command', 0.0), ('drive_seconds', 3.5),
                              ('pause_seconds', 0.3), ('pivot_command', 0.2),
                              ('use_hw1_turns', True), ('stop_margin', 0.05),
                              ('square_to_tag', False), ('pose_timeout', 0.6),
                              ('loss_grace_seconds', 1.5)]:
            self.declare_parameter(name, defaults.get(name, default))
        self.tag_frame = self.get_parameter('tag_frame').value
        self.square_to_tag = self.get_parameter('square_to_tag').value
        self.pose_timeout = self.get_parameter('pose_timeout').value
        if not math.isfinite(self.pose_timeout) or self.pose_timeout <= 0:
            raise ValueError('pose_timeout must be finite and > 0')
        self.policy = self.make_policy()
        self.command_pub = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.measurement = None
        self.last_received = None
        self.source_stamp = None
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.reported_stop = False
        self.reported_aligning = False
        self.last_log = -math.inf
        if self.square_to_tag:
            self.get_logger().info(
                'Square-on mode: target is planar standoff along tag normal; '
                '8 cm position / 3 deg heading tolerance. Scalar stop margin is not applied.')
        self.get_logger().info(
            f'Approach armed for {self.tag_frame}; target {self.policy.target:.2f} m '
            'from robot centre. '
            'Waiting for fresh tag pose.')

    def make_policy(self, target_distance=None):
        policy_type = SquareApproachPolicy if self.square_to_tag else ApproachPolicy
        return policy_type(
            (self.get_parameter('target_distance').value
             if target_distance is None else target_distance),
            self.get_parameter('max_command').value,
            self.get_parameter('max_duration').value,
            min_command=self.get_parameter('min_command').value,
            drive_seconds=self.get_parameter('drive_seconds').value,
            pause_seconds=self.get_parameter('pause_seconds').value,
            pivot_command=self.get_parameter('pivot_command').value,
            use_hw1_turns=self.get_parameter('use_hw1_turns').value,
            stop_margin=self.get_parameter('stop_margin').value,
            loss_grace_seconds=self.get_parameter('loss_grace_seconds').value)

    def on_tf(self, message):
        for tf in message.transforms:
            if tf.header.frame_id != 'camera_frame' or tf.child_frame_id != self.tag_frame:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not 0 <= age <= self.pose_timeout:
                continue
            # Do not let repeated or out-of-order poses refresh the timeout.
            if self.source_stamp is not None and stamp <= self.source_stamp:
                continue
            p = tf.transform.translation
            forward = p.z + CAMERA_OFFSET_FROM_ROBOT_CENTER_M
            distance = math.hypot(p.x, p.y, forward)
            bearing = math.atan2(p.x, forward)
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or p.z <= 0:
                self.policy.stop('Invalid tag position')
                continue
            if self.square_to_tag:
                q = tf.transform.rotation
                try:
                    self.policy.update_goal(p.x, forward, (q.x, q.y, q.z, q.w))
                except ValueError as error:
                    self.policy.stop(str(error))
                    continue
            self.measurement = (distance, bearing)
            self.source_stamp = stamp
            self.last_received = time.monotonic()

    def tick(self):
        now = time.monotonic()
        if self.count_publishers('motor_commands') > 1:
            self.policy.stop('Another motor_commands publisher detected')
        source_age = (self.get_clock().now().nanoseconds * 1e-9 - self.source_stamp
                      if self.source_stamp is not None else math.inf)
        fresh = (self.last_received is not None
                 and now - self.last_received <= self.pose_timeout
                 and 0 <= source_age <= self.pose_timeout)
        distance, bearing = self.measurement or (None, None)
        was_paused = self.policy.paused
        left, right = self.policy.step(now, distance, bearing, fresh)
        self.command_pub.publish(Float32MultiArray(data=[left, right]))
        if self.policy.paused != was_paused and not self.policy.reason:
            self.get_logger().info('Interval pause' if self.policy.paused else 'Interval resumed')
        if self.policy.aligning != self.reported_aligning and not self.policy.reason:
            self.get_logger().info(
                'Aligning in place' if self.policy.aligning else 'Aligned; approaching')
            self.reported_aligning = self.policy.aligning
        if self.policy.reason and not self.reported_stop:
            self.get_logger().info(f'STOP: {self.policy.reason}')
            self.reported_stop = True
        elif fresh and not self.policy.reason and now - self.last_log >= 1.0:
            self.get_logger().info(
                f'range={distance:.3f} m bearing={math.degrees(bearing):+.1f} deg '
                f'L={left:.3f} R={right:.3f} ' +
                (f'goal_error={math.hypot(*self.policy.goal[:2]):.3f} m '
                 f'square_heading={math.degrees(self.policy.goal[2]):+.1f} deg '
                 f'phase={self.policy.phase} ' if self.square_to_tag else '') +
                f'state={"PAUSE" if self.policy.paused else "ALIGN" if self.policy.aligning else "DRIVE"}')
            self.last_log = now


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ApproachTag()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.command_pub.publish(Float32MultiArray(data=[0.0, 0.0]))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
