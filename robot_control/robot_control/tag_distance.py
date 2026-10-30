"""Report robot-centre-to-tag range from fresh AprilTag transforms, in metres."""

import math
import re
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


from robot_control.control_config import CAMERA_FRAME, CAMERA_OFFSET_FROM_ROBOT_CENTER_M

EXPIRY_CHECK_SECONDS = 0.2
POSE_EXPIRY_SECONDS = 1.0
DISTANCE_LOG_SECONDS = 1.0


class TagDistance(Node):
    """Publish distances for all observed tags, or an explicit frame selection."""

    def __init__(self):
        super().__init__('tag_distance')
        self.declare_parameter('camera_frame', CAMERA_FRAME)
        # Empty selection discovers the detector's tag_<id> frames on /tf.
        self.declare_parameter('tag_frames', [''])
        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frames = {
            frame for frame in self.get_parameter('tag_frames').value if frame
        }
        self.publishers_by_frame = {
            frame: self.create_publisher(Float64, f'{frame}/distance', 10)
            for frame in self.tag_frames
        }
        self.last_seen = {}
        self.last_logged = {}
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.on_transforms, qos_profile_sensor_data)
        self.timer = self.create_timer(EXPIRY_CHECK_SECONDS, self.expire)

    def on_transforms(self, message):
        """Translate camera measurements to the robot centre before reporting."""
        now = time.monotonic()
        for transform in message.transforms:
            frame = transform.child_frame_id
            if (transform.header.frame_id != self.camera_frame
                    or (frame not in self.tag_frames if self.tag_frames
                        else re.fullmatch(r'tag_\d+', frame) is None)):
                continue
            position = transform.transform.translation
            forward_depth = position.z + CAMERA_OFFSET_FROM_ROBOT_CENTER_M
            distance = math.hypot(position.x, position.y, forward_depth)
            if not math.isfinite(distance):
                continue
            if frame not in self.publishers_by_frame:
                self.publishers_by_frame[frame] = self.create_publisher(
                    Float64, f'{frame}/distance', 10)
            self.publishers_by_frame[frame].publish(Float64(data=distance))
            self.last_seen[frame] = now
            if now - self.last_logged.get(frame, -math.inf) >= DISTANCE_LOG_SECONDS:
                # Optical camera axes: x right, z forward.
                bearing_deg = math.degrees(math.atan2(position.x, forward_depth))
                self.get_logger().info(
                    f'{frame}: robot-centre-to-tag distance = {distance:.3f} m '
                    f'| bearing = {bearing_deg:+.1f} deg (positive = right) '
                    f'(forward depth = {forward_depth:.3f} m)')
                self.last_logged[frame] = now

    def expire(self):
        """Invalidate the displayed range when a tag has been lost for one second."""
        now = time.monotonic()
        for frame in list(self.last_seen):
            if now - self.last_seen[frame] > POSE_EXPIRY_SECONDS:
                self.publishers_by_frame[frame].publish(Float64(data=math.nan))
                self.get_logger().info(f'{frame}: tag lost; distance unavailable')
                del self.last_seen[frame]


def main(args=None):
    """Run the distance reporter without starting any motor nodes."""
    rclpy.init(args=args)
    node = TagDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
