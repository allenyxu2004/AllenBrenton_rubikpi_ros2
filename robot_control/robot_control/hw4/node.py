"""Follow a preplanned HW4 route using a surveyed AprilTag map."""

from dataclasses import asdict
import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control.hw4.course import read_course
from robot_control.hw4.localization import FixedMapLocalizer
from robot_control.hw4.planning import CourseGrid
from robot_control.report_log import ReportLog
from robot_control.slam.observations import planar_observation, tag_id_from_frame
from robot_control.slam.trajectory import action_to_goal
from robot_control.slam_node import DEFAULTS as HW3_DEFAULTS
from robot_control.version import __version__


PERIOD = 0.05
WATCHDOG = 0.15
STOP = (0.0, 0.0)
DEFAULTS = {key: HW3_DEFAULTS[key] for key in (
    'camera_frame', 'camera_forward_m', 'camera_left_m', 'camera_yaw_rad',
    'forward_left', 'forward_right', 'forward_speed_mps',
    'left_left', 'left_right', 'left_rate_rps',
    'right_left', 'right_right', 'right_rate_rps',
    'settle_s', 'observe_s', 'max_observation_age_s', 'max_tag_range_m',
    'translation_noise', 'innovation_gate', 'route_timeout_s',
)}
DEFAULTS.update(
    heading_tolerance_rad=0.13, rotation_noise=0.15,
    measurement_noise_m=0.08, range_noise_fraction=0.04,
    initial_position_std_m=0.05, initial_heading_std_rad=0.12,
    max_position_std_m=0.25, max_heading_std_rad=0.5,
    max_tag_gap_s=10.0, startup_timeout_s=30.0, log_directory='~/.ros/hw4',
)


class Navigator(Node):
    def __init__(self):
        super().__init__('hw4_navigation')
        self.course = read_course(self.declare_parameter('course', '').value)
        self.objective = self.declare_parameter('objective', 'distance').value
        self.drive = self.declare_parameter('drive', False).value
        self.plan = CourseGrid(self.course).plan(self.objective)
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in DEFAULTS.items()}
        c = self.cfg
        self.validate_settings()
        self.localizer = FixedMapLocalizer(
            self.course, c['translation_noise'], c['rotation_noise'],
            c['initial_position_std_m'], c['initial_heading_std_rad'], c['innovation_gate'])
        self.target = 1
        self.motion = self.wheels = STOP
        self.start = self.previous = self.stopped_at = time.monotonic()
        self.next_decision = self.start + c['settle_s'] + c['observe_s']
        self.pulse_until = self.start
        self.last_tag = self.run_started = None
        self.stamps = {}
        self.finished = False
        self.last_estimate = -math.inf
        self.log = ReportLog(c['log_directory'], self.get_logger(), prefix=f'hw4_{self.objective}')
        self.record('config', software_version=__version__, course=asdict(self.course),
                    objective=self.objective, drive=self.drive, parameters=c,
                    waypoints_m=self.plan.points, grid_distance_m=self.plan.grid_distance,
                    planned_distance_m=self.plan.distance, planned_clearance_m=self.plan.clearance)
        self.commands = self.create_publisher(Float32MultiArray, 'motor_commands', 1) if self.drive else None
        self.pose = self.create_publisher(PoseWithCovarianceStamped, 'hw4/pose', 10)
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.timer = self.create_timer(PERIOD, self.tick)
        self.get_logger().info(
            f'HW4 v{__version__} {self.objective}, drive={self.drive} | '
            f'{self.plan.distance:.2f} m route | {self.plan.clearance:.2f} m minimum clearance')

    def validate_settings(self):
        for key, value in self.cfg.items():
            if not isinstance(value, float):
                continue
            if not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
            if key.startswith('camera_'):
                continue
            if key.endswith(('_left', '_right')):
                if abs(value) > 0.5:
                    raise ValueError(f'{key} exceeds the motor limit')
            elif value <= 0:
                raise ValueError(f'{key} must be positive')
        if self.cfg['settle_s'] < WATCHDOG:
            raise ValueError('settle_s must allow time for the motor bridge to stop')

    def record(self, event, **fields):
        self.log.write(dict(event=event, elapsed_s=time.monotonic() - self.start, **fields))

    def on_tf(self, message):
        if self.motion != STOP or self.finished:
            return
        now = time.monotonic()
        c = self.cfg
        earliest = self.stopped_at + c['settle_s']
        clock_now = self.get_clock().now().nanoseconds * 1e-9
        for tf in message.transforms:
            tag = tag_id_from_frame(tf.child_frame_id)
            if tag not in self.course.landmarks or tf.header.frame_id != c['camera_frame']:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = clock_now - stamp
            if (not 0 <= age <= c['max_observation_age_s'] or now - age < earliest
                    or stamp <= self.stamps.get(tag, -math.inf)):
                continue
            self.stamps[tag] = stamp
            translation = tf.transform.translation
            try:
                forward, left = planar_observation(
                    translation.x, translation.y, translation.z, c['camera_forward_m'],
                    c['camera_left_m'], c['camera_yaw_rad'])
            except ValueError:
                continue
            distance = math.hypot(forward, left)
            if not 0.1 <= distance <= c['max_tag_range_m']:
                continue
            noise = c['measurement_noise_m'] + c['range_noise_fraction'] * distance
            result = self.localizer.observe(tag, forward, left, noise)
            if result == 'updated':
                self.last_tag = now - age
            self.record('observation', tag=tag, result=result,
                        pose=self.localizer.x[:3].tolist())

    def position_std(self):
        return math.sqrt(max(0.0, np.linalg.eigvalsh(self.localizer.P[:2, :2])[-1]))

    def stop(self, now):
        self.motion = self.wheels = STOP
        self.stopped_at = now
        self.next_decision = now + self.cfg['settle_s'] + self.cfg['observe_s']

    def fault(self, now, reason):
        self.stop(now)
        self.publish_stop()
        self.finished = True
        self.record('fault', reason=reason, pose=self.localizer.x[:3].tolist(),
                    run_s=None if self.run_started is None else now - self.run_started)
        self.get_logger().error(reason)

    def tick(self):
        now = time.monotonic()
        dt, self.previous = now - self.previous, now
        if self.motion != STOP:
            self.localizer.predict(*self.motion, min(dt, WATCHDOG))
            if dt >= WATCHDOG - 1e-9:
                self.fault(now, 'Control timing gap while moving')
            elif now >= self.pulse_until:
                self.stop(now)

        if self.drive and not self.finished:
            self.advance(now)
        if self.commands is not None:
            self.commands.publish(Float32MultiArray(data=list(self.wheels)))
        self.publish_pose()
        if now - self.last_estimate >= 0.2:
            self.record('estimate', pose=self.localizer.x[:3].tolist(),
                        position_std_m=self.position_std(), waypoint=self.target)
            self.last_estimate = now
        if self.finished:
            self.timer.cancel()

    def advance(self, now):
        c = self.cfg
        if self.count_publishers('motor_commands') > 1:
            self.fault(now, 'Another motor_commands publisher is active')
            return
        if self.run_started is None:
            if now - self.start > c['startup_timeout_s']:
                self.fault(now, 'Startup timeout waiting for a mapped tag or motor bridge')
            elif (self.commands.get_subscription_count() > 0 and self.last_tag is not None
                  and now - self.last_tag <= c['max_tag_gap_s'] and now >= self.next_decision):
                self.run_started = now
                self.record('run_start')
            else:
                return
        if now - self.run_started > c['route_timeout_s']:
            self.fault(now, 'Run timeout')
        elif now - self.last_tag > c['max_tag_gap_s']:
            self.fault(now, 'Mapped tag observations went stale')
        elif self.commands.get_subscription_count() == 0:
            self.fault(now, 'Motor bridge subscriber disappeared')
        elif (self.position_std() > c['max_position_std_m']
              or self.localizer.P[2, 2] > c['max_heading_std_rad']**2):
            self.fault(now, 'Pose uncertainty exceeded configured limit')
        elif self.motion == STOP and now >= self.next_decision:
            self.decide(now)

    def decide(self, now):
        c = self.cfg
        x, y, theta = self.localizer.x[:3]
        minimum = self.course.robot_radius + self.course.margin
        if self.course.clearance((x, y), (x, y)) <= minimum:
            self.fault(now, 'Estimated pose entered the reserved clearance margin')
            return
        while self.target < len(self.plan.points):
            goal = self.plan.points[self.target]
            action, error = action_to_goal((x, y, theta), goal, self.course.waypoint_tolerance,
                                           c['heading_tolerance_rad'])
            if action != 'arrived':
                break
            self.target += 1
        else:
            self.finished = True
            self.publish_stop()
            self.record('complete', run_s=now - self.run_started, pose=[x, y, theta])
            self.get_logger().info('Goal reached')
            return

        if self.course.clearance((x, y), goal) <= minimum:
            self.fault(now, 'Tracking deviation blocks a safe approach to the waypoint')
            return
        if action == 'forward':
            speed = c['forward_speed_mps']
            ticks = max(1, min(3, round(error / (speed * PERIOD))))
            # Include the watchdog interval in case the next stop command is delayed.
            for ticks in range(ticks, 0, -1):
                reach = speed * (ticks * PERIOD + WATCHDOG)
                end = (x + reach * math.cos(theta), y + reach * math.sin(theta))
                if self.course.clearance((x, y), end) > minimum:
                    break
            else:
                self.fault(now, 'Predicted drive pulse would enter the reserved clearance margin')
                return
            self.motion = (speed, 0.0)
        else:
            rate = c[f'{action}_rate_rps']
            ticks = max(1, min(3, round(error / (rate * PERIOD))))
            self.motion = (0.0, rate if action == 'left' else -rate)
        self.wheels = (c[f'{action}_left'], c[f'{action}_right'])
        self.pulse_until = now + (ticks - 0.5) * PERIOD
        self.record('pulse', action=action, ticks=ticks, waypoint=self.target, wheels=self.wheels)

    def publish_stop(self):
        if self.commands is not None and rclpy.ok():
            self.commands.publish(Float32MultiArray(data=list(STOP)))

    def publish_pose(self):
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'hw4_map'
        x, y, theta = self.localizer.x[:3]
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        message.pose.pose.orientation.z = math.sin(theta / 2)
        message.pose.pose.orientation.w = math.cos(theta / 2)
        for i, row in enumerate((0, 1, 5)):
            for j, column in enumerate((0, 1, 5)):
                message.pose.covariance[row * 6 + column] = float(self.localizer.P[i, j])
        for axis in (2, 3, 4):
            message.pose.covariance[axis * 6 + axis] = 1e6
        self.pose.publish(message)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = Navigator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.record('shutdown')
            node.log.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
