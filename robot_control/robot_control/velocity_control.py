"""Timed open-loop waypoint control"""

from dataclasses import dataclass
import math
import time


# Motor commands are firmware units, not physical wheel speeds.
FORWARD_LEFT = 0.36
FORWARD_RIGHT = 0.2
FORWARD_SPEED = 44 * 0.0254 / 4  # m/s
LEFT_TURN_LEFT = -0.45
LEFT_TURN_RIGHT = 0.3
LEFT_TURN_RATE = math.pi / 2 / 0.738  # rad/s magnitude
RIGHT_TURN_LEFT = 0.42
RIGHT_TURN_RIGHT = -0.3
# Assumes the two right-turn trials produced 180 degrees TOTAL.
RIGHT_TURN_RATE = math.pi / (0.740 + 0.738)
MAX_SEGMENT_SECONDS = 4.0
PUBLISH_PERIOD = 0.05  # 20 Hz
STOP_SECONDS = 0.3
POSITION_TOLERANCE = 0.01  # m, applied to estimated pose only
HEADING_TOLERANCE = 0.01  # rad, applied to estimated pose only


@dataclass(frozen=True)
class Waypoint:
    # target pose in the world frame: x, y in meters, theta in radians (CCW positive)
    x: float
    y: float
    theta: float


def load_waypoints(path):
    # read "x,y,theta" lines; blank lines and lines starting with '#' are skipped
    waypoints = []
    with open(path) as f:
        for line_num, line in enumerate(f, start=1):
            line = line.strip()
            if not line or line.startswith('#'):
                continue

            fields = [field.strip() for field in line.split(',')]
            if len(fields) != 3:
                raise ValueError(
                    f'{path}:{line_num}: expected "x,y,theta", got "{line}"')

            try:
                x, y, theta = (float(field) for field in fields)
            except ValueError:
                raise ValueError(
                    f'{path}:{line_num}: non-numeric value in "{line}"') from None

            if not all(math.isfinite(value) for value in (x, y, theta)):
                raise ValueError(f'{path}:{line_num}: coordinates must be finite')

            waypoints.append(Waypoint(x, y, theta))

    if not waypoints:
        raise ValueError(f'{path}: no waypoints found')

    return waypoints


def normalize_angle(angle):
    """Return the shortest signed angle in [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


@dataclass
class EstimatedPose:
    """Pose integrated from commanded motion, never a measured pose."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    def integrate(self, speed, turn_rate, elapsed):
        """Integrate the differential-drive model for straight/pivot motion."""
        self.x += speed * math.cos(self.theta) * elapsed
        self.y += speed * math.sin(self.theta) * elapsed
        self.theta = normalize_angle(self.theta + turn_rate * elapsed)


def run_route(node, publisher, waypoints):
    """Execute timed segments while servicing ROS and refreshing commands."""
    import rclpy
    from std_msgs.msg import Float32MultiArray

    pose = EstimatedPose()

    def send(left, right):
        message = Float32MultiArray()
        message.data = [float(left), float(right)]
        publisher.publish(message)

    def timed_motion(left, right, duration, speed=0.0, turn_rate=0.0):
        start = time.monotonic()
        deadline = start + duration
        previous = start
        try:
            while rclpy.ok():
                now = time.monotonic()
                pose.integrate(speed, turn_rate, now - previous)
                previous = now
                if now >= deadline:
                    break
                send(left, right)
                rclpy.spin_once(
                    node, timeout_sec=min(PUBLISH_PERIOD, deadline - now))
            if not rclpy.ok():
                raise InterruptedError('ROS context shut down')
        finally:
            if rclpy.ok():
                send(0.0, 0.0)

    def pause():
        timed_motion(0.0, 0.0, STOP_SECONDS)

    def turn_to(target):
        while abs(normalize_angle(target - pose.theta)) > HEADING_TOLERANCE:
            error = normalize_angle(target - pose.theta)
            if error > 0:
                left, right, rate = (
                    LEFT_TURN_LEFT, LEFT_TURN_RIGHT, LEFT_TURN_RATE)
            else:
                left, right, rate = (
                    RIGHT_TURN_LEFT, RIGHT_TURN_RIGHT, -RIGHT_TURN_RATE)
            duration = min(abs(error / rate), MAX_SEGMENT_SECONDS)
            timed_motion(left, right, duration, turn_rate=rate)
            pause()

    try:
        node.get_logger().info('Waiting for a motor_commands subscriber...')
        while rclpy.ok() and publisher.get_subscription_count() == 0:
            send(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=PUBLISH_PERIOD)
        if not rclpy.ok():
            return
        pause()

        for index, waypoint in enumerate(waypoints, start=1):
            node.get_logger().info(
                f'Waypoint {index}/{len(waypoints)}: '
                f'({waypoint.x}, {waypoint.y}, {waypoint.theta})')
            while True:
                dx, dy = waypoint.x - pose.x, waypoint.y - pose.y
                distance = math.hypot(dx, dy)
                if distance <= POSITION_TOLERANCE:
                    break
                turn_to(math.atan2(dy, dx))
                duration = min(distance / FORWARD_SPEED, MAX_SEGMENT_SECONDS)
                timed_motion(
                    FORWARD_LEFT, FORWARD_RIGHT, duration, speed=FORWARD_SPEED)
                pause()
            turn_to(waypoint.theta)
            node.get_logger().info(
                f'Estimated pose: x={pose.x:.3f} m, y={pose.y:.3f} m, '
                f'heading={pose.theta:.3f} rad')
        node.get_logger().info('Route complete (open-loop estimate).')
    finally:
        if rclpy.ok():
            # Repeat zero commands long enough for the serial bridge to send them.
            pause()


def main(args=None):
    """Run the installed waypoint file, or a ROS waypoint_file override."""
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from pathlib import Path
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Float32MultiArray

    # Keep the context alive on Ctrl-C so the finally block can publish stops.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('waypoint_controller')
    try:
        default_path = str(
            Path(get_package_share_directory('robot_control'))
            / 'config' / 'waypoints.txt')
        node.declare_parameter('waypoint_file', default_path)
        waypoints = load_waypoints(node.get_parameter('waypoint_file').value)
        publisher = node.create_publisher(
            Float32MultiArray, 'motor_commands', 10)
        run_route(node, publisher, waypoints)
    except (KeyboardInterrupt, InterruptedError):
        node.get_logger().info('Route interrupted; stopping.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
