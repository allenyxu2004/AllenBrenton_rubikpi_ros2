"""ROS orchestration: predict motion, observe while settled, follow waypoints.

The math, optical conversion, and waypoint policy are separate pure modules.
This node owns timing and is the only HW3 publisher of motor_commands.
"""

import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control.slam.ekf import EkfSlam
from robot_control.slam.observations import planar_observation, tag_id_from_frame
from robot_control.slam.trajectory import action_to_goal, action_to_heading, polygon_waypoints


DEFAULTS = dict(
    drive=False, shape='square', side_m=0.9, laps=1,
    camera_frame='camera_frame', camera_forward_m=0.0889,
    camera_left_m=0.0, camera_yaw_rad=0.0,
    forward_left=0.25, forward_right=0.2, forward_speed_mps=0.29845,
    left_left=-0.45, left_right=0.3, left_rate_rps=1.588267,
    right_left=0.45, right_right=-0.3, right_rate_rps=1.828518,
    pulse_s=0.15, settle_s=0.35, observe_s=0.25, lap_observe_s=1.0,
    position_tolerance_m=0.08, heading_tolerance_rad=0.10,
    max_observation_age_s=0.6, route_timeout_s=600.0,
    max_position_std_m=0.6, max_heading_std_rad=0.8,
    max_tag_range_m=4.0, measurement_noise_m=0.04,
    range_noise_fraction=0.02, translation_noise=0.08, rotation_noise=0.20,
    innovation_gate=9.21, log_directory='~/.ros/hw3_slam',
)


class SlamNode(Node):
    """Pulse, settle, observe, then decide again from the corrected pose."""

    def __init__(self):
        super().__init__('hw3_slam')
        self.cfg = {name: self.declare_parameter(name, value).value
                    for name, value in DEFAULTS.items()}
        self.validate_config()
        c = self.cfg
        self.filter = EkfSlam(c['translation_noise'], c['rotation_noise'],
                              c['measurement_noise_m'], c['innovation_gate'])
        self.goals = polygon_waypoints(c['shape'], c['side_m'], c['laps'])
        self.corner = 0
        self.corners_per_lap = 4 if c['shape'] == 'square' else 8
        self.lap_phase = None  # At the closing corner: align, then observe.
        self.lap_observe_until = 0.0
        self.finished = False
        self.fault = None
        self.stamps = {}
        self.start = self.previous = time.monotonic()
        self.stopped_at = self.start
        self.next_decision = self.start + c['settle_s'] + c['observe_s']
        self.pulse_until = 0.0
        self.motion = (0.0, 0.0)
        self.wheels = (0.0, 0.0)
        self.last_snapshot = -math.inf
        directory = Path(c['log_directory']).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        # Exclusive creation protects earlier experiment results.
        self.log_path = directory / f'{time.time_ns()}_{c["shape"]}.jsonl'
        self.log = self.log_path.open('x', buffering=1)
        self.record('config', parameters=c)
        self.commands = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.poses = self.create_publisher(PoseWithCovarianceStamped, 'slam/pose', 10)
        self.tags = self.create_subscription(TFMessage, '/tf', self.on_tf,
                                             qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info(f'Log: {self.log_path}; drive={c["drive"]}')

    def validate_config(self):
        """Fail before publishing commands if a parameter is unsafe/meaningless."""
        signed = {'camera_forward_m', 'camera_left_m', 'camera_yaw_rad',
                  'forward_left', 'forward_right', 'left_left', 'left_right',
                  'right_left', 'right_right'}
        for key, default in DEFAULTS.items():
            value = self.cfg[key]
            if isinstance(default, float):
                if not math.isfinite(value) or (key not in signed and value <= 0):
                    raise ValueError(f'Invalid parameter {key}: {value}')
        for key in ('forward_left', 'forward_right', 'left_left', 'left_right',
                    'right_left', 'right_right'):
            if abs(self.cfg[key]) > 0.5:
                raise ValueError(f'{key} exceeds motor limit')
        if self.cfg['settle_s'] < 0.15:
            raise ValueError('settle_s must allow serial transmission and stopping')

    def record(self, event, **fields):
        self.log.write(json.dumps(dict(event=event, elapsed_s=time.monotonic() - self.start,
                                       **fields), allow_nan=False) + '\n')

    def predict_to(self, now):
        """Integrate the previous command before replacing it.

        After a long callback gap the bridge watchdog has stopped the wheels;
        integrate at most its 0.15 s timeout and latch a fault during motion.
        """
        dt = max(0.0, now - self.previous)
        if dt > 0.15 and self.motion != (0.0, 0.0):
            self.fault = 'Control timing gap; restart the experiment'
        self.filter.predict(*self.motion, min(dt, 0.15))
        self.previous = now

    def on_tf(self, message):
        now = time.monotonic()
        self.predict_to(now)
        c = self.cfg
        # Delayed images taken during a pulse must never update a settled pose.
        if self.motion != (0.0, 0.0) or now - self.stopped_at < c['settle_s']:
            return
        clock_now = self.get_clock().now().nanoseconds * 1e-9
        for tf in message.transforms:
            tag = tag_id_from_frame(tf.child_frame_id)
            if tag is None or tf.header.frame_id != c['camera_frame']:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = clock_now - stamp
            if not 0 <= age <= c['max_observation_age_s']:
                continue
            if now - age < self.stopped_at + c['settle_s']:
                continue
            if stamp <= self.stamps.get(tag, -math.inf):
                continue
            self.stamps[tag] = stamp
            p = tf.transform.translation
            try:
                forward, left = planar_observation(
                    p.x, p.y, p.z, c['camera_forward_m'],
                    c['camera_left_m'], c['camera_yaw_rad'])
            except ValueError:
                continue
            distance = math.hypot(forward, left)
            if not 0.1 <= distance <= c['max_tag_range_m']:
                continue
            noise = c['measurement_noise_m'] + c['range_noise_fraction'] * distance
            result = self.filter.observe(tag, forward, left, noise)
            self.record('observation', tag=tag, forward=forward, left=left,
                        source_stamp_s=stamp, result=result)

    def publish_stop(self):
        """Also used during shutdown; serial bridge supplies a second watchdog."""
        self.commands.publish(Float32MultiArray(data=[0.0, 0.0]))

    def tick(self):
        now = time.monotonic()
        self.predict_to(now)
        c = self.cfg
        if self.motion != (0.0, 0.0) and (now >= self.pulse_until or self.fault):
            self.motion, self.wheels = (0.0, 0.0), (0.0, 0.0)
            self.stopped_at = now
            self.next_decision = now + c['settle_s'] + c['observe_s']
        if self.count_publishers('motor_commands') > 1:
            self.fault = 'Another motor_commands publisher is active'
        if c['drive'] and not self.finished:
            if now - self.start > c['route_timeout_s']:
                self.fault = 'Route timeout'
            P = self.filter.P
            if (max(P[0, 0], P[1, 1]) > c['max_position_std_m']**2
                    or P[2, 2] > c['max_heading_std_rad']**2):
                self.fault = 'Pose uncertainty exceeded configured limit'
        if self.fault:
            if not self.finished:
                self.get_logger().error(self.fault)
                self.record('fault', reason=self.fault)
            self.finished = True
            self.motion, self.wheels = (0.0, 0.0), (0.0, 0.0)
        elif (c['drive'] and not self.finished
              and self.commands.get_subscription_count() > 0
              and self.motion == (0.0, 0.0) and now >= self.next_decision):
            self.decide(now)
        self.commands.publish(Float32MultiArray(data=list(self.wheels)))
        self.publish_pose()
        if now - self.last_snapshot >= 0.2:
            self.record('estimate', state=self.filter.x.tolist(),
                        covariance=self.filter.P.tolist(),
                        landmarks=self.filter.indices, corner=self.corner,
                        wheels=self.wheels, finished=self.finished)
            self.last_snapshot = now

    def decide(self, now):
        c = self.cfg
        if self.lap_phase == 'observe':
            if now < self.lap_observe_until:
                return
            # Keep the corrections without commanding another return-to-start
            # maneuver. The observation pause completes this lap.
            # Observations during the pause are now included in this lap's
            # arrival snapshot, which the offline evaluator uses for errors.
            self.lap_phase = None
            self.finish_corner()
            return
        if self.lap_phase == 'align':
            # Startup defines heading zero. Rotate without chasing position
            # corrections while trying to reacquire the starting view.
            action, error = action_to_heading(
                self.filter.x[2], 0.0, c['heading_tolerance_rad'])
        else:
            action, error = action_to_goal(
                self.filter.x[:3], self.goals[self.corner],
                c['position_tolerance_m'], c['heading_tolerance_rad'])
        if action == 'arrived':
            if (self.corner + 1) % self.corners_per_lap == 0:
                if self.lap_phase is None:
                    self.lap_phase = 'align'
                    self.get_logger().info('Lap closing: returning to startup heading')
                else:
                    # Each pivot already ends with settle_s + observe_s. This
                    # extra stationary window gives the starting tags time to
                    # update SLAM before the lap is declared complete.
                    self.lap_phase = 'observe'
                    self.lap_observe_until = now + c['lap_observe_s']
                    self.next_decision = self.lap_observe_until
                    self.record('lap_observation_start', corner=self.corner + 1)
                    self.get_logger().info('Startup heading reached; observing before lap completion')
            else:
                self.finish_corner()
            return
        if action == 'forward':
            self.motion = (c['forward_speed_mps'], 0.0)
            self.wheels = (c['forward_left'], c['forward_right'])
            duration = error / c['forward_speed_mps']
        else:
            rate = c[f'{action}_rate_rps']
            self.motion = (0.0, rate if action == 'left' else -rate)
            self.wheels = (c[f'{action}_left'], c[f'{action}_right'])
            duration = error / rate
        # Quantize to the 20 Hz command period. Shorter pulses are not reliably
        # delivered by the bridge, so heading tolerance must allow one tick.
        duration = max(0.05, min(c['pulse_s'], duration))
        self.pulse_until = now + duration
        self.record('command', action=action, wheels=self.wheels,
                    speed=self.motion[0], turn_rate=self.motion[1], duration_s=duration)

    def finish_corner(self):
        """Record arrival after any end-of-lap alignment and observations."""
        self.corner += 1
        self.record('arrival', corner=self.corner, state=self.filter.x.tolist(),
                    covariance=self.filter.P.tolist(), landmarks=self.filter.indices)
        self.get_logger().info(f'Corner {self.corner}/{len(self.goals)} reached')
        if self.corner == len(self.goals):
            self.finished = True
            self.record('complete')
            self.get_logger().info('Route complete; remaining stopped')

    def publish_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'slam_map'
        x, y, theta = self.filter.x[:3]
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(theta / 2)
        message.pose.pose.orientation.w = math.cos(theta / 2)
        axes = (0, 1, 5)
        for i, row in enumerate(axes):
            for j, column in enumerate(axes):
                message.pose.covariance[row * 6 + column] = float(self.filter.P[i, j])
        # z, roll and pitch are not observed by this planar filter.
        for axis in (2, 3, 4):
            message.pose.covariance[axis * 6 + axis] = 1e6
        self.poses.publish(message)


def main(args=None):
    """Keep ROS alive through Ctrl-C so a final zero command can be sent."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = SlamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_stop()
            node.record('shutdown', state=node.filter.x.tolist(),
                        covariance=node.filter.P.tolist(), landmarks=node.filter.indices)
            node.log.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
