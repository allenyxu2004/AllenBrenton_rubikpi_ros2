"""Sequence the existing approach_tag controller through the requested route."""
import math
import time
import rclpy
from std_msgs.msg import Float32MultiArray
from robot_control.approach_tag import ApproachTag
from robot_control.tag_route_steps import AlignTagPolicy, STAGES, ALIGN_COMPLETE


class TagRoute(ApproachTag):
    def __init__(self):
        self.stage_index = 0
        self.resume_at = 0.
        self.finished = False
        super().__init__('tag_route', defaults={
            'tag_frame': 'tag_9', 'target_distance': 0.5,
            'square_to_tag': True, 'min_command': 0.08,
            'max_command': 0.15, 'max_duration': 60.0,
            'use_hw1_turns': True, 'drive_seconds': 3.5,
            'pause_seconds': 0.3})
        self.get_logger().info('Route: approach 9, align 8, approach 8, align 7, '
                               'approach 7, align 5, approach 5, align 9, approach 9 to 1.5 m, stop')

    def tick(self):
        if self.finished or time.monotonic() < self.resume_at:
            self.command_pub.publish(Float32MultiArray(data=[0., 0.]))
            return
        # All pose freshness, approach timing, safety checks and publishing
        # remain in the working ApproachTag implementation.
        super().tick()
        reason = self.policy.reason
        arrived = (reason == ALIGN_COMPLETE or
                   (isinstance(reason, str) and
                    reason.startswith('Square-on goal reached:')))
        if not arrived:
            return
        self.stage_index += 1
        if self.stage_index == len(STAGES):
            self.finished = True
            self.get_logger().info('Route complete; remaining stopped 1.5 m from tag 9')
            return
        kind, tag, distance = STAGES[self.stage_index]
        self.tag_frame = f'tag_{tag}'
        self.square_to_tag = kind == 'approach'
        if self.square_to_tag:
            self.policy = self.make_policy(target_distance=distance)
        else:
            self.policy = AlignTagPolicy(
                target_distance=self.get_parameter('target_distance').value,
                max_command=self.get_parameter('max_command').value,
                max_duration=30.,
                min_command=self.get_parameter('min_command').value,
                drive_seconds=self.get_parameter('drive_seconds').value,
                pause_seconds=self.get_parameter('pause_seconds').value,
                pivot_command=self.get_parameter('pivot_command').value,
                use_hw1_turns=self.get_parameter('use_hw1_turns').value,
                loss_grace_seconds=self.get_parameter('loss_grace_seconds').value)
        self.measurement = self.last_received = self.source_stamp = None
        self.reported_stop = self.reported_aligning = False
        self.last_log = -math.inf
        self.resume_at = time.monotonic()+self.get_parameter('pause_seconds').value
        self.get_logger().info(f'Next route step: {kind} {self.tag_frame}; target={self.policy.target:.2f} m')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TagRoute()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.command_pub.publish(Float32MultiArray(data=[0., 0.]))
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
