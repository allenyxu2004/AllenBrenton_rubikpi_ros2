"""ROS adapter for the tested HW4 planning and pulse-navigation core."""

from dataclasses import asdict
import math
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage

from robot_control.hw4.control import DEFAULTS, NavigationCore, PERIOD, STOP, valid_estimate
from robot_control.hw4.course import read_course
from robot_control.report_log import ReportLog
from robot_control.version import __version__


class Navigator(Node):
    def __init__(self):
        super().__init__('hw4_navigation')
        course = read_course(self.declare_parameter('course', '').value)
        objective = self.declare_parameter('objective', 'safety').value
        drive = self.declare_parameter('drive', False).value
        settings = {key: self.declare_parameter(key, value).value for key, value in DEFAULTS.items()}
        self.start = time.monotonic()
        self.log = ReportLog(settings['log_directory'], self.get_logger(), prefix=f'hw4_{objective}')
        self.record('config', software_version=__version__, schema_version=2, course=asdict(course),
                    objective=objective, drive=drive, parameters=settings,
                    planning_reserve_m=course.padding)
        # drive=false remains the default; drive=true explicitly enables commands.
        self.core = NavigationCore(course, objective, drive, settings, self.start, self.record)
        self.commands = self.create_publisher(Float32MultiArray, 'motor_commands', 1) if drive else None
        self.pose = self.create_publisher(PoseWithCovarianceStamped, 'hw4/pose', 10)
        self.path = self.create_publisher(Path, 'hw4/path',
                                          QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_generation = 0
        self.last_estimate = -math.inf
        self.subscription = self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.timer = self.create_timer(PERIOD, self.tick)
        self.get_logger().info(f'HW4 v{__version__} | {objective} | drive={drive} | planning while stopped')

    def record(self, event, **fields):
        def finite_json(value):
            if isinstance(value, float) and not math.isfinite(value):
                return None
            if isinstance(value, dict):
                return {key: finite_json(item) for key, item in value.items()}
            if isinstance(value, (list, tuple)):
                return [finite_json(item) for item in value]
            return value

        self.log.write(finite_json(dict(event=event, elapsed_s=time.monotonic() - self.start, **fields)))
        if event == 'fault':
            self.get_logger().error(fields['reason'])
        elif event == 'plan':
            self.get_logger().info(f"Plan {fields['generation']}: {fields['planner']}, "
                                   f"{fields['distance']:.3f} m, body clearance {fields['clearance']:.3f} m")
        elif event == 'complete':
            self.get_logger().info('Goal reached after pulse/settle/observe')

    def on_tf(self, message):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        try:
            for tf in message.transforms:
                self.core.observe_tf(tf, now, ros_now)
        except Exception as error:
            self.core.fault(now, f'Observation callback failed: {error}')
            self.publish_stop()

    def tick(self):
        now = time.monotonic()
        try:
            ready = self.commands is not None and self.commands.get_subscription_count() > 0
            wheels = self.core.step(now, ready, self.count_publishers('motor_commands'))
            if self.commands is not None:
                self.commands.publish(Float32MultiArray(data=list(wheels)))
                self.record('command', wheels=wheels, generation=self.core.generation)
            self.publish_pose()
            if self.core.plan is not None and self.path_generation != self.core.generation:
                self.publish_path()
            if now - self.last_estimate >= 0.2:
                self.record('estimate', pose=self.core.localizer.x[:3].tolist(),
                            covariance=self.core.localizer.P[:3, :3].tolist(),
                            waypoint=self.core.target, generation=self.core.generation)
                self.last_estimate = now
        except Exception as error:
            self.core.fault(now, f'Control callback failed: {error}')
            self.publish_stop()
        # Keep this timer alive after completion/fault to publish continuous zeros.

    def publish_stop(self):
        if self.commands is not None and rclpy.ok():
            self.commands.publish(Float32MultiArray(data=list(STOP)))

    def publish_path(self):
        message = Path()
        message.header.frame_id = 'hw4_map'
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.core.plan.points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self.path.publish(message)
        self.path_generation = self.core.generation

    def publish_pose(self):
        if not valid_estimate(self.core.localizer):
            return
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'hw4_map'
        x, y, theta = self.core.localizer.x[:3]
        message.pose.pose.position.x, message.pose.pose.position.y = float(x), float(y)
        message.pose.pose.orientation.z = math.sin(theta / 2)
        message.pose.pose.orientation.w = math.cos(theta / 2)
        for i, row in enumerate((0, 1, 5)):
            for j, column in enumerate((0, 1, 5)):
                message.pose.covariance[row * 6 + column] = float(self.core.localizer.P[i, j])
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
            node.core.close()
            node.record('shutdown')
            node.log.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
