"""Run route steps using the same ROS controller as standalone approach_tag."""
import time

import rclpy

from robot_control.approach_tag import ApproachTag
from robot_control.route_config import ALIGNMENT_TIMEOUT_SECONDS, ROUTE_DEFAULTS, STAGES
from robot_control.tag_route_steps import ALIGN_COMPLETE, AlignTagPolicy


class TagRoute(ApproachTag):
    """Add target sequencing around ApproachTag's existing control loop."""

    def __init__(self):
        """Start with tag 9 and the proven approach settings in route_config."""
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
        reason = self.policy.reason
        return (reason == ALIGN_COMPLETE or
                (isinstance(reason, str) and
                 reason.startswith('Square-on goal reached:')))

    def _advance_route(self):
        """Stop at the end, or prepare the next target after a short pause."""
        self.stage_index += 1
        if self.stage_index == len(STAGES):
            self.finished = True
            self.get_logger().info('Route complete; remaining stopped 1.5 m from tag 9')
            return

        step = STAGES[self.stage_index]
        self.tag_frame = f'tag_{step.tag_id}'
        self.square_to_tag = step.action == 'approach'
        if self.square_to_tag:
            # None uses the normal 0.5 m target; the last step overrides to 1.5 m.
            self.policy = self.make_policy(target_distance=step.distance)
        else:
            self.policy = self._make_alignment_policy()

        self.reset_tracking()
        self.resume_at = time.monotonic() + self.get_parameter('pause_seconds').value
        self.get_logger().info(
            f'Next route step: {step.action} {self.tag_frame}; '
            f'target={self.policy.target:.2f} m')

    def _make_alignment_policy(self):
        """Use the same motion settings for turn-only search and centering."""
        parameter_names = (
            'target_distance', 'max_command', 'min_command', 'drive_seconds',
            'pause_seconds', 'pivot_command', 'use_hw1_turns', 'loss_grace_seconds')
        settings = {name: self.get_parameter(name).value for name in parameter_names}
        return AlignTagPolicy(max_duration=ALIGNMENT_TIMEOUT_SECONDS, **settings)


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
