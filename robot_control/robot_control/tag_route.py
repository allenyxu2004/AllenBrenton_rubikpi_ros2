"""Run route steps using the same ROS controller as standalone approach_tag."""
import time

import rclpy

from robot_control.approach_tag import ApproachTag
from robot_control.route_config import ALIGNMENT_TIMEOUT_SECONDS, ROUTE_DEFAULTS, STAGES
from robot_control.tag_route_steps import AlignTagPolicy
from robot_control.approach_policy import Outcome


class TagRoute(ApproachTag):
    """Add target sequencing around ApproachTag's existing control loop."""

    def __init__(self):
        """Start the configured first stage using the route motion settings."""
        self.stage_index = 0
        self.resume_at = 0.0
        self.finished = False
        super().__init__('tag_route', defaults=ROUTE_DEFAULTS)
        route_description = ', '.join(
            f'{step.action} {step.tag_id}' for step in STAGES)
        self.get_logger().info(f'Route: {route_description}, stop')

    def tick(self):
        """Run shared control, then advance only after a successful step."""
        if self.finished or time.monotonic() < self.resume_at:
            self.publish_command()
            return

        # Keep freshness checks, motor safety, command publishing and logging
        # in ApproachTag. The route does not duplicate that control logic.
        super().tick()
        if self._step_succeeded():
            self._advance_route()

    def _step_succeeded(self):
        """Distinguish arrival from latched errors; errors never skip a target."""
        return self.policy.outcome is Outcome.SUCCEEDED

    def _advance_route(self):
        """Stop at the end, or prepare the next target after a short pause."""
        self.stage_index += 1
        if self.stage_index == len(STAGES):
            self.report_event('complete')
            self.finished = True
            self.get_logger().info(
                f'Route complete; stopped after {STAGES[-1].action} {self.tag_frame}; '
                f'target={self.policy.target:.2f} m')
            return

        self._start_stage(STAGES[self.stage_index])
        self.resume_at = time.monotonic() + self.get_parameter('pause_seconds').value
        self.get_logger().info(
            f'Next route step: {STAGES[self.stage_index].action} {self.tag_frame}; '
            f'target={self.policy.target:.2f} m')

    def initialize_policy(self):
        """Initialize from STAGES, just as for every later transition."""
        if not STAGES:
            raise ValueError('Route must contain at least one stage')
        self._start_stage(STAGES[0])

    def _start_stage(self, step):
        """Apply one configured target, action and optional standoff."""
        if step.action not in ('approach', 'align'):
            raise ValueError(f'Unknown route action: {step.action}')
        self.tag_frame = f'tag_{step.tag_id}'
        self.square_to_tag = step.action == 'approach'
        if self.square_to_tag:
            self.policy = self.make_policy(target_distance=step.distance)
        else:
            settings = self.policy_settings()
            settings['max_duration'] = ALIGNMENT_TIMEOUT_SECONDS
            if step.distance is not None:
                settings['target_distance'] = step.distance
            self.policy = AlignTagPolicy(**settings)
        self.reset_tracking()


def main(args=None):
    """Spin the route controller and send a stop command during shutdown."""
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
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
