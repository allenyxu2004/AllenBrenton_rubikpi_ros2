"""Generate polygons and choose one calibrated straight/pivot action."""

import math

from robot_control.kinematic_control import normalize_angle


def polygon_waypoints(shape, side, laps):
    """Start at origin facing +x; traverse a regular polygon CCW.

    side means edge length for BOTH shapes. Route lies left of the initial
    heading and extends behind the starting x for an octagon.
    """
    if shape not in ('square', 'octagon'):
        raise ValueError('shape must be square or octagon')
    if not math.isfinite(side) or side <= 0 or laps < 1:
        raise ValueError('Positive side length and at least one lap required')
    count = 4 if shape == 'square' else 8
    x, y = 0.0, 0.0
    corners = []
    for edge in range(count):
        angle = edge * 2 * math.pi / count
        x += side * math.cos(angle)
        y += side * math.sin(angle)
        corners.append((x, y))
    return corners * laps


def action_to_goal(pose, goal, position_tolerance, heading_tolerance):
    """Use the SLAM pose, not elapsed route time, to select each new pulse."""
    dx, dy = goal[0] - pose[0], goal[1] - pose[1]
    distance = math.hypot(dx, dy)
    if distance <= position_tolerance:
        return 'arrived', distance
    action, error = action_to_heading(pose[2], math.atan2(dy, dx), heading_tolerance)
    if action != 'arrived':
        return action, error
    return 'forward', distance


def action_to_heading(heading, target, tolerance):
    """Select a pivot toward a target heading, including across +/-pi."""
    error = normalize_angle(target - heading)
    if abs(error) <= tolerance:
        return 'arrived', abs(error)
    return ('left' if error > 0 else 'right'), abs(error)
