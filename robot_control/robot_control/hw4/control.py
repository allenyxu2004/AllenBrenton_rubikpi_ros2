"""ROS-independent pulse navigation, bounded planning, and swept safety guards."""

from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import math
from threading import Event

import numpy as np

from robot_control.hw4.course import EPS, point_segment_distance
from robot_control.hw4.localization import FixedMapLocalizer
from robot_control.hw4.planning import plan_course
from robot_control.slam.observations import planar_observation, tag_id_from_frame
from robot_control.slam.trajectory import action_to_goal

PERIOD = 0.05
WATCHDOG = 0.15
STOP_DELAY = WATCHDOG + PERIOD  # Watchdog expiry plus the bridge's next serial tick.
STOP = (0.0, 0.0)
# Values copied explicitly from the prior HW4 runtime. HW3 behavior is unchanged.
DEFAULTS = dict(
    camera_frame='camera_frame', camera_forward_m=0.0889, camera_left_m=0.0, camera_yaw_rad=0.0,
    forward_left=0.25, forward_right=0.20, forward_speed_mps=0.219070,
    left_left=-0.45, left_right=0.30, left_rate_rps=2.117690,
    right_left=0.45, right_right=-0.30, right_rate_rps=1.969739,
    settle_s=0.35, observe_s=0.25, max_observation_age_s=0.6, max_tag_range_m=4.0,
    translation_noise=0.08, rotation_noise=0.15, innovation_gate=9.21,
    measurement_noise_m=0.08, range_noise_fraction=0.04,
    initial_position_std_m=0.05, initial_heading_std_rad=0.12,
    max_position_std_m=0.10, max_heading_std_rad=0.20, heading_tolerance_rad=0.10,
    max_tag_gap_s=2.0, startup_timeout_s=30.0, route_timeout_s=600.0,
    planning_timeout_s=15.0, max_replans=3, goal_confirmation_timeout_s=5.0,
    min_pulse_s=0.05, max_pulse_s=0.15,
    coast_distance_m=0.01, lateral_error_m=0.01, pivot_drift_m=0.01,
    log_directory='~/.ros/hw4',
)


def validate_settings(cfg):
    for key, default in DEFAULTS.items():
        value = cfg[key]
        if isinstance(default, str):
            if not isinstance(value, str) or not value:
                raise ValueError(f'{key} must be nonempty text')
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{key} must be finite numeric data')
        if key.startswith('camera_'):
            continue
        if key.endswith(('_left', '_right')):
            if abs(value) > 0.5:
                raise ValueError(f'{key} exceeds the motor command bound')
        elif value < 0 or (value == 0 and key not in (
                'coast_distance_m', 'lateral_error_m', 'pivot_drift_m', 'max_replans')):
            raise ValueError(f'{key} must be positive (error bounds/replan count may be zero)')
    if type(cfg['max_replans']) is not int:
        raise ValueError('max_replans must be an integer')
    if cfg['settle_s'] < WATCHDOG:
        raise ValueError('Settling must include the motor watchdog interval')
    if not PERIOD <= cfg['min_pulse_s'] <= cfg['max_pulse_s'] <= WATCHDOG:
        raise ValueError('Pulse durations must be between one command tick and the watchdog')
    for key in ('min_pulse_s', 'max_pulse_s'):
        if abs(cfg[key] / PERIOD - round(cfg[key] / PERIOD)) > EPS:
            raise ValueError('Pulse durations must be whole command ticks')
    if cfg['max_tag_gap_s'] <= cfg['settle_s'] + cfg['observe_s'] + cfg['max_pulse_s']:
        raise ValueError('Tag timeout must permit one pulse/settle/observe cycle')


def position_std(covariance):
    return math.sqrt(max(0.0, float(np.linalg.eigvalsh(covariance[:2, :2])[-1])))


def valid_estimate(localizer):
    covariance = localizer.P[:3, :3]
    return (np.isfinite(localizer.x).all() and np.isfinite(localizer.P).all()
            and np.allclose(covariance, covariance.T, atol=1e-10)
            and np.linalg.eigvalsh(covariance)[0] >= -1e-10)


@dataclass(frozen=True)
class Envelope:
    safe: bool
    reason: str
    end: tuple[float, float]
    duration_s: float
    position_bound_m: float
    heading_std_rad: float
    required_clearance_m: float
    available_clearance_m: float
    motion_reserve_used_m: float


def pulse_envelope(course, localizer, cfg, action, duration):
    """Enclose the complete straight sweep or pivot disc, including a delayed stop.

    The positional variance bound holds throughout [0,T], using covariance
    cross-term norms and an upper bound on integrated heading process noise.
    Thus checking the exact entire segment with that radius also covers between
    command ticks. This is a model bound, not a bound on unmeasured physical bias.
    """
    x, y, theta = localizer.x[:3]
    speed = cfg['forward_speed_mps'] if action == 'forward' else 0.0
    coast = cfg['coast_distance_m'] if speed else 0.0
    total = duration + STOP_DELAY + (coast / speed if speed else 0.0)
    distance = speed * total
    p = localizer.P[:3, :3]
    variance = (position_std(p) ** 2 + 2 * distance * float(np.linalg.norm(p[:2, 2]))
                + distance ** 2 * max(0.0, p[2, 2]) + cfg['translation_noise'] ** 2 * total
                + (speed * cfg['rotation_noise']) ** 2 * total ** 3 / 3)
    uncertainty = 3 * math.sqrt(max(0.0, float(variance)))
    heading = math.sqrt(max(0.0, float(p[2, 2])) + cfg['rotation_noise'] ** 2 * total)
    drift = cfg['lateral_error_m'] if speed else cfg['pivot_drift_m']
    reserve = speed * STOP_DELAY + coast + drift
    end = (float(x + distance * math.cos(theta)), float(y + distance * math.sin(theta)))
    required = course.robot_radius + course.margin + uncertainty + drift
    available = course.clearance((x, y), end)
    reason = ''
    if uncertainty > course.uncertainty_budget:
        reason = 'Predicted uncertainty exceeds its planning budget'
    elif reserve > course.motion_reserve:
        reason = 'Delayed stop/coast/drift exceeds its planning budget'
    elif heading > cfg['max_heading_std_rad']:
        reason = 'Predicted heading uncertainty exceeds its limit'
    elif available <= required:
        reason = 'Complete swept envelope reaches a hazard or engineering margin'
    return Envelope(not reason, reason, end, total, uncertainty, heading, required, available, reserve)


class NavigationCore:
    """Single-thread-owned control state; only immutable planning runs in a worker."""

    def __init__(self, course, objective='safety', drive=False, settings=None,
                 now=0.0, record=None, executor=None):
        self.course = course
        self.objective = objective
        if objective not in ('safety', 'distance'):
            raise ValueError('objective must be safety or distance')
        self.drive = drive
        self.cfg = dict(DEFAULTS, **(settings or {}))
        validate_settings(self.cfg)
        if drive:
            course.require_drive_ready()
        c = self.cfg
        self.localizer = FixedMapLocalizer(course, c['translation_noise'], c['rotation_noise'],
                                           c['initial_position_std_m'], c['initial_heading_std_rad'],
                                           c['innovation_gate'])
        self.record = record or (lambda event, **fields: None)
        self.executor = executor or ThreadPoolExecutor(max_workers=1, thread_name_prefix='hw4-plan')
        self.start = self.previous = self.stopped_at = now
        self.next_decision = now + c['settle_s'] + c['observe_s']
        self.motion = self.wheels = STOP
        self.finished = False
        self.outcome = None
        self.last_tag = self.run_started = self.goal_pending = None
        self.plan = self.future = None
        self.generation = 0
        self.replans = 0
        self.target = 1
        self.stamps = {}
        self.cancel = Event()
        self.request_plan(now, 'startup')

    def stop(self, now):
        self.motion = self.wheels = STOP
        self.stopped_at = now
        self.next_decision = now + self.cfg['settle_s'] + self.cfg['observe_s']

    def fault(self, now, reason):
        if self.finished:
            return
        self.stop(now)
        self.cancel.set()
        self.finished = True
        self.outcome = 'fault'
        self.record('fault', reason=reason, pose=self.localizer.x[:3].tolist(),
                    generation=self.generation,
                    run_s=None if self.run_started is None else now - self.run_started)

    def request_plan(self, now, reason):
        if self.future is not None or self.finished:
            return
        if self.generation:
            self.replans += 1
            if self.replans > self.cfg['max_replans']:
                self.fault(now, 'Replanning limit reached')
                return
        self.stop(now)
        self.plan = None
        self.generation += 1
        self.goal_pending = None
        try:
            snapshot = replace(self.course, start=tuple(self.localizer.x[:2]),
                               landmarks=dict(self.course.landmarks))
        except ValueError as error:
            self.fault(now, f'Corrected start cannot be planned safely: {error}')
            return
        self.cancel = Event()
        self.planning_at = now
        self.planning_start = snapshot.start
        self.record('planning', generation=self.generation, reason=reason, start_m=snapshot.start)
        self.future = self.executor.submit(plan_course, deepcopy(snapshot), self.objective,
                                           self.cfg['planning_timeout_s'], self.cancel)

    def poll_plan(self, now):
        if self.future is None or self.finished:
            return
        if now - self.planning_at > self.cfg['planning_timeout_s']:
            self.fault(now, 'Planning deadline exceeded')
            return
        if not self.future.done():
            return
        future, self.future = self.future, None
        try:
            plan = future.result()
        except Exception as error:
            self.fault(now, f'Planning failed: {error}')
            return
        point = tuple(self.localizer.x[:2])
        target = plan.points[min(1, len(plan.points) - 1)]
        if (math.dist(point, self.planning_start) > self.course.tracking_allowance
                or not self.course.feasible(point, target)):
            self.record('plan_discarded', generation=self.generation, reason='Corrected start/connector changed')
            self.request_plan(now, 'stale start connection')
            return
        self.plan = plan
        self.target = min(1, len(plan.points) - 1)
        self.record('plan', generation=self.generation, objective=self.objective, **asdict(plan))

    def observe_tf(self, tf, now, ros_now):
        """Accept only distinct mapped observations captured after settling."""
        if self.motion != STOP or self.finished:
            return
        c = self.cfg
        tag = tag_id_from_frame(tf.child_frame_id)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        age = ros_now - stamp
        reason = None
        if tag not in self.course.landmarks:
            reason = 'unknown_tag'
        elif tf.header.frame_id != c['camera_frame']:
            reason = 'wrong_frame'
        # Floating conversion of sec/nanosec can differ by a fraction of a
        # microsecond at Unix epochs; this is not a future-frame allowance.
        elif not math.isfinite(age) or not -1e-6 <= age <= c['max_observation_age_s']:
            reason = 'stale_or_future'
        elif now - age < self.stopped_at + c['settle_s']:
            reason = 'before_settling'
        elif stamp <= self.stamps.get(tag, -math.inf):
            reason = 'duplicate_or_out_of_order'
        if reason:
            self.record('observation_rejected', tag=tag, reason=reason)
            return
        self.stamps[tag] = stamp
        translation = tf.transform.translation
        try:
            forward, left = planar_observation(translation.x, translation.y, translation.z,
                                               c['camera_forward_m'], c['camera_left_m'], c['camera_yaw_rad'])
            if not 0.1 <= math.hypot(forward, left) <= c['max_tag_range_m']:
                raise ValueError('out of range')
        except ValueError:
            self.record('observation_rejected', tag=tag, reason='invalid_translation_or_range')
            return
        noise = c['measurement_noise_m'] + c['range_noise_fraction'] * math.hypot(forward, left)
        result = self.localizer.observe(tag, forward, left, noise)
        if result == 'updated':
            self.last_tag = now - max(0.0, age)
        self.record('observation', tag=tag, result=result, source_stamp_s=stamp,
                    pose=self.localizer.x[:3].tolist(), covariance=self.localizer.P[:3, :3].tolist(),
                    diagnostics=self.localizer.last_diagnostics)

    def step(self, now, bridge_ready=True, publishers=1):
        """Return wheels every command tick, including zeros after a latched stop."""
        dt, self.previous = now - self.previous, now
        if self.finished:
            return STOP
        if not valid_estimate(self.localizer):
            self.fault(now, 'Invalid pose or covariance')
            return STOP
        if dt < 0 or (self.motion != STOP and dt >= WATCHDOG - EPS):
            self.fault(now, 'Control timing gap while moving or monotonic clock reversal')
            return STOP
        if self.motion != STOP:
            self.localizer.predict(*self.motion, dt)
            if not valid_estimate(self.localizer):
                self.fault(now, 'Invalid prediction')
                return STOP
            self.record('prediction', dt_s=dt, motion=self.motion, pose=self.localizer.x[:3].tolist())
            if now >= self.pulse_until:
                self.stop(now)
        self.poll_plan(now)
        if not self.drive or self.finished:
            return STOP
        c = self.cfg
        if publishers > 1:
            self.fault(now, 'Another motor_commands publisher is active')
        elif self.run_started is None:
            if now - self.start > c['startup_timeout_s']:
                self.fault(now, 'Startup timeout waiting for plan, mapped tag or motor bridge')
            elif (bridge_ready and self.plan is not None and self.last_tag is not None
                  and now - self.last_tag <= c['max_tag_gap_s'] and now >= self.next_decision):
                self.run_started = now
                self.record('run_start', generation=self.generation)
        elif now - self.run_started > c['route_timeout_s']:
            self.fault(now, 'Route timeout')
        elif not bridge_ready:
            self.fault(now, 'Motor bridge subscriber disappeared')
        elif self.last_tag is None or now - self.last_tag > c['max_tag_gap_s']:
            self.fault(now, 'Mapped tag observations went stale')
        elif (position_std(self.localizer.P) > c['max_position_std_m']
              or self.localizer.P[2, 2] > c['max_heading_std_rad'] ** 2):
            self.fault(now, 'Pose uncertainty exceeded its global limit')
        elif self.motion == STOP and self.plan is not None and now >= self.next_decision:
            self.decide(now)
        return self.wheels

    def decide(self, now):
        c = self.cfg
        point = tuple(self.localizer.x[:2])
        radius = self.course.robot_radius + self.course.margin + 3 * position_std(self.localizer.P)
        if self.course.clearance(point, point) <= radius + c['pivot_drift_m']:
            self.fault(now, 'Corrected pose uncertainty reaches a hazard')
            return
        goal = self.plan.points[self.target]
        action, error = action_to_goal(self.localizer.x[:3], goal,
                                       self.course.waypoint_tolerance, c['heading_tolerance_rad'])
        if self.goal_pending is not None:
            if (self.last_tag is not None and self.last_tag >= self.stopped_at + c['settle_s']
                    and action == 'arrived'):
                self.finished = True
                self.outcome = 'complete'
                self.wheels = self.motion = STOP
                self.record('complete', run_s=now - self.run_started,
                            generation=self.generation, pose=self.localizer.x[:3].tolist())
            elif now - self.goal_pending > c['goal_confirmation_timeout_s']:
                self.fault(now, 'No fresh stopped goal confirmation')
            elif action != 'arrived':
                self.goal_pending = None
            return
        if action == 'arrived':
            if self.target == len(self.plan.points) - 1:
                self.goal_pending = now
                self.stop(now)
                self.record('goal_confirmation', generation=self.generation)
            else:
                following = self.plan.points[self.target + 1]
                if self.course.clearance(point, following) <= radius + c['pivot_drift_m']:
                    self.request_plan(now, 'Waypoint tolerance would cut a blocked corner')
                    return
                self.record('waypoint', target=self.target, generation=self.generation)
                self.target += 1  # Never skip several corners in one decision.
            return
        previous = self.plan.points[max(0, self.target - 1)]
        if point_segment_distance(point, previous, goal) > self.course.tracking_allowance:
            self.request_plan(now, 'Tracking allowance exceeded')
            return
        if self.course.clearance(point, goal) <= radius + c['pivot_drift_m']:
            self.request_plan(now, 'Corrected waypoint connector blocked')
            return
        speed = c['forward_speed_mps'] if action == 'forward' else c[f'{action}_rate_rps']
        minimum = round(c['min_pulse_s'] / PERIOD)
        maximum = round(c['max_pulse_s'] / PERIOD)
        ticks = max(minimum, min(maximum, round(error / (speed * PERIOD))))
        for ticks in range(ticks, minimum - 1, -1):
            envelope = pulse_envelope(self.course, self.localizer, c, action, ticks * PERIOD)
            self.record('envelope', generation=self.generation, action=action, **asdict(envelope))
            if envelope.safe:
                break
        else:
            self.fault(now, f'No deliverable safe pulse: {envelope.reason}')
            return
        self.motion = (speed, 0.0) if action == 'forward' else (0.0, speed if action == 'left' else -speed)
        self.wheels = (c[f'{action}_left'], c[f'{action}_right'])
        # The timer publishes on a tick; allow roundoff, never sub-tick pulses.
        self.pulse_until = now + ticks * PERIOD - EPS
        self.record('pulse', action=action, ticks=ticks, duration_s=ticks * PERIOD,
                    waypoint=self.target, wheels=self.wheels, generation=self.generation)

    def close(self):
        self.cancel.set()
        if self.future is not None:
            self.future.cancel()
        self.executor.shutdown(wait=False, cancel_futures=True)
