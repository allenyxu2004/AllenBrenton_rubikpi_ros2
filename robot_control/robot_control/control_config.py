"""Shared controller defaults and physical camera mounting configuration."""

CAMERA_FRAME = 'camera_frame'
# Forward-facing camera, with no mounting rotation or lateral/vertical offset.
CAMERA_OFFSET_FROM_ROBOT_CENTER_M = 3.5 * 0.0254
CONTROL_PERIOD_SECONDS = 0.05
PROGRESS_LOG_SECONDS = 1.0
MIN_QUATERNION_NORM = 1e-6

POLICY_DEFAULTS = {
    'target_distance': 1.0,
    'max_command': 0.08,
    'max_duration': 20.0,
    'min_command': 0.0,
    'drive_seconds': 3.5,
    'pause_seconds': 0.3,
    'pivot_command': 0.2,
    'use_hw1_turns': False,
    'stop_margin': 0.05,
    'loss_grace_seconds': 0.0,
}
NODE_DEFAULTS = {
    **POLICY_DEFAULTS,
    'use_hw1_turns': True,
    'loss_grace_seconds': 1.5,
    'report_log_directory': '~/.ros/route_reports',
    'tag_frame': 'tag_0',
    'square_to_tag': False,
    'pose_timeout': 0.6,
}
