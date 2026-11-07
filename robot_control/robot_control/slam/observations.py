"""Convert optical camera coordinates into planar robot measurements."""

import math


def tag_id_from_frame(frame):
    """Support HW2 names (tag_9) and upstream names (tag36h11:9)."""
    suffix = frame[4:] if frame.startswith('tag_') else frame.split(':')[-1]
    if not frame.startswith(('tag_', 'tag36h11:', '36h11:')):
        return None
    return int(suffix) if suffix.isdecimal() else None


def planar_observation(x, y, z, forward_offset, left_offset, yaw):
    """Optical +z is forward, +x is right, +y is down.

    Assumes a level camera. Height is discarded, NOT included in planar range.
    yaw rotates camera forward toward robot left; offsets are in robot axes.
    """
    if not all(math.isfinite(v) for v in (x, y, z)) or z <= 0:
        raise ValueError('Invalid camera translation')
    # First change optical axes: q = [camera_z, -camera_x]^T.
    # Then rotate by mounting yaw psi and translate to the robot center:
    #
    # [ forward ]   [ offset_forward ]   [ cos(psi) -sin(psi) ] [  z ]
    # [  left   ] = [ offset_left    ] + [ sin(psi)  cos(psi) ] [ -x ]
    #
    # camera_y is vertical, so it does not enter this ground-plane equation.
    c, s = math.cos(yaw), math.sin(yaw)
    return forward_offset + c * z + s * x, left_offset + s * z - c * x
