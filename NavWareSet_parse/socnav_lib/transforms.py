"""GRS <-> robot map frame transform.

The x_grs_to_bot_offset.json files give {x, y, z, yaw_deg}. Fitted against
scene 1 (Umeyama rigid-transform fit of 1_robot_and_participants.csv's
robot_x/y/yaw, GRS frame, against 1_robot_pose.csv's x/y/yaw_rad, robot/map
frame, matched through the GRS<->robot clock offset below):

    p_map = R(-yaw_deg) @ p_grs - (offset.x, offset.y)
    angle_map = angle_grs - radians(yaw_deg)

Residual on scene 1: ~0.12 m mean position error, ~3.6 deg median heading
error (heading is noisier since both tracks are independent estimates).
offset.z is a height offset between the two sensor mounts and is unused
here since we only work with 2D poses.
"""

import numpy as np

# robot_bag_epoch_ns - grs_bag_epoch_ns, fitted on scene 1 by aligning
# 1_robot_pose.csv against 1_robot_and_participants.csv (matches the
# constant already used ad-hoc in ros_parse_node.py).
GRS_TO_ROBOT_CLOCK_OFFSET_NS = 15_058_455_523


def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class GrsToMapTransform:
    def __init__(self, offset: dict):
        yaw = -np.radians(offset["yaw_deg"])
        c, s = np.cos(yaw), np.sin(yaw)
        self._R = np.array([[c, -s], [s, c]])
        self._t = -np.array([offset["x"], offset["y"]])
        self._yaw = yaw

    def point(self, x: float, y: float) -> tuple:
        p = self._R @ np.array([x, y]) + self._t
        return float(p[0]), float(p[1])

    def angle(self, a: float) -> float:
        return float(wrap_angle(a + self._yaw))
