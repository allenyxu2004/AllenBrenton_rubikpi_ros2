"""Deterministic visual route sequencing without ROS dependencies."""
import math
from robot_control.approach_policy import ApproachPolicy
from robot_control.kinematic_control import LEFT_TURN_LEFT, LEFT_TURN_RIGHT, FORWARD_LEFT, FORWARD_RIGHT


class RoutePolicy:
    stages = [('approach', 9), ('align', 8), ('approach', 8),
              ('align', 7), ('align', 5), ('approach', 5), ('align', 9)]

    def __init__(self, target_distance=0.5, max_command=0.08,
                 alignment_tolerance=3.0, search_timeout=30.0,
                 approach_timeout=60.0, loss_grace=1.5,
                 drive_seconds=3.5, pause_seconds=0.3):
        values = (target_distance, max_command, alignment_tolerance,
                  search_timeout, approach_timeout, loss_grace)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Route parameters must be finite and positive')
        if max_command > 0.3:
            raise ValueError('max_command must be <= 0.3')
        self.target_distance, self.max_command = target_distance, max_command
        self.tolerance = math.radians(alignment_tolerance)
        self.search_timeout, self.approach_timeout = search_timeout, approach_timeout
        self.loss_grace = loss_grace
        self.index = 0
        self.reason = None
        self.state = 'WAIT'
        self.started = None
        self.settled = None
        self.lost = None
        self.seen = False
        self.pause_until = 0
        self.drive_seconds = drive_seconds
        self.pause_seconds = pause_seconds
        self.policy = self._motion_policy()
        self.search_pulse = None
        self.search_pause_until = 0.

    def _motion_policy(self):
        return ApproachPolicy(
            target_distance=self.target_distance, max_command=self.max_command,
            max_duration=self.approach_timeout, drive_seconds=self.drive_seconds,
            pause_seconds=self.pause_seconds, use_hw1_turns=True,
            stop_margin=0.0, loss_grace_seconds=self.loss_grace)

    def _search(self, now):
        if self.search_pulse is not None:
            if now-self.search_pulse >= 0.1:
                self.search_pulse = None
                self.search_pause_until = now+0.2
                return 0., 0.
            return LEFT_TURN_LEFT, LEFT_TURN_RIGHT
        if now < self.search_pause_until:
            return 0., 0.
        self.search_pulse = now
        return LEFT_TURN_LEFT, LEFT_TURN_RIGHT

    @property
    def target(self):
        return self.stages[min(self.index, len(self.stages)-1)][1]

    def stop(self, reason):
        self.reason = reason
        self.state = 'FAILED'
        return 0., 0.

    def step(self, now, distance=None, bearing=None, fresh=False):
        if self.reason or self.state == 'DONE':
            return 0., 0.
        if now < self.pause_until:
            return 0., 0.
        if self.started is None:
            self.started = now
        kind, _ = self.stages[self.index]
        deadline = self.approach_timeout if kind == 'approach' and self.seen else self.search_timeout
        if now-self.started >= deadline:
            return self.stop('Stage timeout')
        if fresh and not all(v is not None and math.isfinite(v) for v in (distance, bearing)):
            return self.stop('Invalid pose')
        if not fresh:
            self.settled = None
            self.policy.pivot_started_at = None
            if self.seen:
                self.lost = now if self.lost is None else self.lost
                if now-self.lost >= self.loss_grace:
                    return self.stop('Target lost')
                return 0., 0.
            self.state = 'WAIT' if self.index == 0 else 'SEARCH'
            return self._search(now) if self.index else (0., 0.)
        self.seen = True
        self.lost = None
        if abs(bearing) > self.tolerance:
            self.settled = None
            self.state = 'CENTER'
            self.policy.aligning = True
            command = self.policy.step(
                now, self.target_distance+1., bearing, True)
            if self.policy.reason:
                return self.stop(self.policy.reason)
            return command
        if kind == 'approach' and distance > self.target_distance:
            self.state = 'APPROACH'
            command = self.policy.step(now, distance, bearing, True)
            if self.policy.reason:
                return self.stop(self.policy.reason)
            left, right = command
            if left > 0 and right > 0:
                left *= FORWARD_LEFT/FORWARD_RIGHT
                scale = min(1., self.max_command/max(left, right))
                left, right = left*scale, right*scale
            return left, right
        self.state = 'SETTLE'
        self.settled = now if self.settled is None else self.settled
        if now-self.settled >= 0.5:
            self.index += 1
            if self.index == len(self.stages):
                self.state = 'DONE'
            else:
                self.started = None
                self.seen = False
                self.settled = None
                self.pause_until = now+self.pause_seconds
                self.policy = self._motion_policy()
                self.search_pulse = None
                self.search_pause_until = 0.
        return 0., 0.
