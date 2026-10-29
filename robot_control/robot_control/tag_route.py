"""Run the configured AprilTag route using fresh visual feedback."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray
from tf2_msgs.msg import TFMessage
from robot_control.tag_route_policy import RoutePolicy
from robot_control.tag_distance import CAMERA_OFFSET_FROM_ROBOT_CENTER_M


class TagRoute(Node):
    def __init__(self):
        super().__init__('tag_route')
        defaults = dict(target_distance=0.5, max_command=0.08,
                        alignment_tolerance=3.0, search_timeout=30.0,
                        approach_timeout=60.0, loss_grace=1.5,
                        drive_seconds=3.5, pause_seconds=0.3)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.policy = RoutePolicy(**{k: self.get_parameter(k).value for k in defaults})
        self.declare_parameter('pose_timeout', 0.6)
        self.timeout = self.get_parameter('pose_timeout').value
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('pose_timeout must be positive')
        self.poses = {}
        self.pub = self.create_publisher(Float32MultiArray, 'motor_commands', 1)
        self.sub = self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.last_log = None

    def on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id != 'camera_frame':
                continue
            frame = tf.child_frame_id
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
            age = self.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= self.timeout:
                continue
            if frame in self.poses and stamp <= self.poses[frame][0]:
                continue
            p = tf.transform.translation
            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or p.z <= 0:
                if frame == f'tag_{self.policy.target}':
                    self.policy.stop('Invalid target position')
                continue
            z = p.z+CAMERA_OFFSET_FROM_ROBOT_CENTER_M
            self.poses[frame] = (stamp, time.monotonic(), math.hypot(p.x, p.y, z), math.atan2(p.x, z))

    def tick(self):
        now = time.monotonic()
        if self.count_publishers('motor_commands') > 1:
            self.policy.stop('Another motor command publisher detected')
        pose = self.poses.get(f'tag_{self.policy.target}')
        fresh = (pose is not None and now-pose[1] <= self.timeout and
                 0 <= self.get_clock().now().nanoseconds*1e-9-pose[0] <= self.timeout)
        command = self.policy.step(now, pose[2] if pose else None,
                                   pose[3] if pose else None, fresh)
        self.pub.publish(Float32MultiArray(data=list(command)))
        status = (self.policy.index, self.policy.state, self.policy.reason)
        if status != self.last_log:
            self.get_logger().info(f'tag={self.policy.target} state={self.policy.state} reason={self.policy.reason}')
            self.last_log = status


def main(args=None):
    rclpy.init(args=args)
    node = TagRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Float32MultiArray(data=[0., 0.]))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
