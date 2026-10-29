"""Deterministic visual route sequencing without ROS dependencies."""
import math
from robot_control.approach_policy import ApproachPolicy
from robot_control.kinematic_control import LEFT_TURN_LEFT, LEFT_TURN_RIGHT, FORWARD_LEFT, FORWARD_RIGHT


class RoutePolicy:
    stages = [('approach', 9), ('align', 8), ('approach', 8),
              ('align', 7), ('align', 5), ('approach', 5), ('align', 9)]

    def __init__(self, target_distance=0.5, max_command=0.08,
                 alignment_tolerance=3.0, search_timeout=30.0,
                 approach_timeout=60.0, loss_grace=1.5):
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
        self.policy = None

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
            if self.seen:
                self.lost = now if self.lost is None else self.lost
                if now-self.lost >= self.loss_grace:
                    return self.stop('Target lost')
                return 0., 0.
            self.state = 'WAIT' if self.index == 0 else 'SEARCH'
            return ((LEFT_TURN_LEFT, LEFT_TURN_RIGHT)
                    if self.index and (now-self.started) % 0.3 < 0.1 else (0., 0.))
        self.seen = True
        self.lost = None
        if abs(bearing) > self.tolerance:
            self.settled = None
            self.state = 'CENTER'
            if (now-self.started) % 0.3 >= 0.1:
                return 0., 0.
            from robot_control.kinematic_control import RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT
            return ((RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT) if bearing > 0
                    else (LEFT_TURN_LEFT, LEFT_TURN_RIGHT))
        if kind == 'approach' and distance > self.target_distance:
            self.state = 'APPROACH'
            speed = min(self.max_command, 0.25*(distance-self.target_distance))
            turn = max(-speed*0.3, min(speed*0.3, 0.12*bearing))
            left, right = (speed+turn)*FORWARD_LEFT/FORWARD_RIGHT, speed-turn
            scale = min(1., self.max_command/max(left, right))
            return left*scale, right*scale
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
                self.pause_until = now+0.3
        return 0., 0.
