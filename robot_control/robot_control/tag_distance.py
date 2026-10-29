"""Report robot-centre-to-tag range from fresh AprilTag transforms, in metres."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


# Camera faces forward, 3.5 inches ahead of the robot centre.
# Assume zero lateral/vertical offset and no camera mounting rotation.
CAMERA_OFFSET_FROM_ROBOT_CENTER_M = 3.5 * 0.0254  # 0.0889 m


class TagDistance(Node):
    """Publish distances for the configured detector's tag frames."""

    def __init__(self):
        super().__init__('tag_distance')
        self.declare_parameter('camera_frame', 'camera_frame')
        self.declare_parameter('tag_frames', ['tag_0', 'tag_1'])
        self.camera_frame = self.get_parameter('camera_frame').value
        self.publishers_by_frame = {
            frame: self.create_publisher(Float64, f'{frame}/distance', 10)
            for frame in self.get_parameter('tag_frames').value
        }
        self.last_seen = {}
        self.last_logged = {}
        self.subscription = self.create_subscription(
            TFMessage, '/tf', self.on_transforms, qos_profile_sensor_data)
        self.timer = self.create_timer(0.2, self.expire)

    def on_transforms(self, message):
        """Translate camera measurements to the robot centre before reporting."""
        now = time.monotonic()
        for transform in message.transforms:
            frame = transform.child_frame_id
            if (transform.header.frame_id != self.camera_frame
                    or frame not in self.publishers_by_frame):
                continue
            position = transform.transform.translation
            forward_depth = position.z + CAMERA_OFFSET_FROM_ROBOT_CENTER_M
            distance = math.hypot(position.x, position.y, forward_depth)
            if not math.isfinite(distance):
                continue
            self.publishers_by_frame[frame].publish(Float64(data=distance))
            self.last_seen[frame] = now
            if now - self.last_logged.get(frame, -math.inf) >= 1.0:
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
            if now - self.last_seen[frame] > 1.0:
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
