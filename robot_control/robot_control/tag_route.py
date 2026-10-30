"""Run the configured AprilTag route, publishing wheel commands at 20 Hz."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control import route_config as cfg
from robot_control.report_log import ReportLog
from robot_control.tag_route_policy import AlignTagPolicy, Outcome, SquareApproachPolicy


class TagRoute(Node):
    def __init__(self):
        super().__init__('tag_route')
        if not math.isfinite(cfg.POSE_TIMEOUT_SECONDS) or cfg.POSE_TIMEOUT_SECONDS <= 0:
            raise ValueError('Pose timeout must be finite and positive')
        if not cfg.STAGES:
            raise ValueError('Route must contain at least one stage')
        self.stage_index = 0
        self.resume_at = 0.0
        self.finished = False
        self.started_at = time.monotonic()
        self.report_stamps = {}
        self.report = ReportLog(cfg.REPORT_DIRECTORY, self.get_logger())
        self.command_pub = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.start_stage()
        self.report_event('start', camera_forward_m=cfg.CAMERA_OFFSET_FROM_ROBOT_CENTER_M)
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.timer = self.create_timer(cfg.CONTROL_PERIOD_SECONDS, self.tick)

    def start_stage(self):
        step = cfg.STAGES[self.stage_index]
        if step.action not in ('approach', 'align'):
            raise ValueError(f'Unknown route action: {step.action}')
        self.tag_frame = f'tag_{step.tag_id}'
        self.approaching = step.action == 'approach'
        policy_type = SquareApproachPolicy if self.approaching else AlignTagPolicy
        target = cfg.TARGET_DISTANCE_M if step.distance is None else step.distance
        self.policy = policy_type(target)
        self.measurement = None
        self.last_received = None
        self.source_stamp = None
        self.observation_time = None
        self.reported_stop = False
        self.last_log = -math.inf
        self.get_logger().info(
            f'Stage {self.stage_index + 1}: {step.action} {self.tag_frame}, target={target:g} m')

    def report_event(self, event, **fields):
        self.report.write(dict(event=event, elapsed_s=time.monotonic() - self.started_at,
                               stage=self.stage_index + 1, tag=self.tag_frame, **fields))

    def on_tf(self, message):
        for tf in message.transforms:
            frame = tf.child_frame_id
            if tf.header.frame_id != cfg.CAMERA_FRAME or not frame.startswith('tag_'):
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if (not 0 <= age <= cfg.POSE_TIMEOUT_SECONDS
                    or stamp <= self.report_stamps.get(frame, -math.inf)):
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            position, quaternion = [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]
            if (not all(math.isfinite(v) for v in position + quaternion) or p.z <= 0
                    or sum(v * v for v in quaternion) < cfg.MIN_QUATERNION_NORM ** 2):
                if frame == self.tag_frame:
                    self.policy.stop('Invalid tag pose')
                continue
            self.report_stamps[frame] = stamp
            self.report_event('observation', observed_tag=frame, source_stamp_s=stamp,
                              translation_m=position, quaternion_xyzw=quaternion)
            if frame != self.tag_frame:
                continue
            forward = p.z + cfg.CAMERA_OFFSET_FROM_ROBOT_CENTER_M
            if self.approaching:
                try:
                    self.policy.update_goal(p.x, forward, quaternion)
                except ValueError as error:
                    self.policy.stop(str(error))
                    continue
            self.measurement = (math.hypot(p.x, p.y, forward), math.atan2(p.x, forward))
            self.source_stamp = stamp
            self.last_received = time.monotonic()
            # Compare capture time with turn settling on the same clock.
            source_age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            self.observation_time = self.last_received - source_age

    def pose_is_fresh(self, now):
        source_age = (self.get_clock().now().nanoseconds * 1e-9 - self.source_stamp
                      if self.source_stamp is not None else math.inf)
        return (self.last_received is not None
                and now - self.last_received <= cfg.POSE_TIMEOUT_SECONDS
                and 0 <= source_age <= cfg.POSE_TIMEOUT_SECONDS)

    def publish_command(self, left=0.0, right=0.0):
        self.command_pub.publish(Float32MultiArray(data=[left, right]))

    def tick(self):
        now = time.monotonic()
        if self.finished or now < self.resume_at:
            self.publish_command()
            return
        if self.count_publishers('motor_commands') > 1:
            self.policy.stop('Another motor_commands publisher detected')
        fresh = self.pose_is_fresh(now)
        distance, bearing = self.measurement or (None, None)
        left, right = self.policy.step(now, bearing, fresh, self.observation_time)
        self.publish_command(left, right)
        control = dict(left_command=left, right_command=right, fresh=fresh,
                       source_stamp_s=self.source_stamp, bearing_rad=bearing,
                       pose_age_s=(None if self.observation_time is None
                                   else now - self.observation_time),
                       waiting_after_turn=self.policy.turn_pause_until is not None)
        goal_log = ''
        if self.approaching:
            control['phase'] = self.policy.phase
            control['goal_right_forward_heading'] = self.policy.goal
            if self.policy.goal is not None:
                gx, gz, heading = self.policy.goal
                goal_log = (f' phase={self.policy.phase} '
                            f'goal_right={gx:+.3f} m goal_forward={gz:+.3f} m '
                            f'goal_heading={math.degrees(heading):+.1f} deg')
        # Record every command; the 1 Hz console can miss short pivot pulses.
        self.report_event('control', **control)
        if self.policy.reason:
            if not self.reported_stop:
                self.get_logger().info(f'STOP: {self.policy.reason}')
                self.report_event('stop', reason=self.policy.reason, **control)
                self.reported_stop = True
            if self.policy.outcome is Outcome.SUCCEEDED:
                self.advance_route()
        elif fresh and now - self.last_log >= cfg.PROGRESS_LOG_SECONDS:
            self.get_logger().info(
                f'{self.tag_frame}: range={distance:.3f} m '
                f'bearing={math.degrees(bearing):+.1f} deg L={left:.3f} R={right:.3f}'
                f'{goal_log}')
            self.last_log = now

    def advance_route(self):
        if self.approaching:
            right, forward, heading = self.policy.goal
            self.report_event('arrival', source_stamp_s=self.source_stamp,
                              position_error_m=math.hypot(right, forward),
                              heading_error_deg=math.degrees(heading),
                              target_distance_m=self.policy.target)
        if self.stage_index + 1 == len(cfg.STAGES):
            self.report_event('complete')
            self.finished = True
            self.get_logger().info('Route complete; remaining stopped')
            return
        self.stage_index += 1
        self.start_stage()
        self.resume_at = time.monotonic() + cfg.PAUSE_SECONDS


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
                node.publish_command()
            node.report.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
