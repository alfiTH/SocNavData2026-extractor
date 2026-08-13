#!/usr/bin/env python3
"""Stage 1 (needs ROS): pulls the single /static_obstacle_ros_map message
out of a scene's <n>_robot.bag and derives wall segments from it via
contour extraction. Writes experiments/<scene>/<scene>_static_map.json.

The robot type (hsr/jackal) is auto-detected from the bag's topic names
(socnav_lib.robot_config.detect_robot_type); pass --robot-type to override
it. Either way it's just a key into socnav_lib/robot_configs.json, the
shared, hand-editable file that holds each platform's footprint/drive type
-- edit that file, not this script, to fix a robot's dimensions.

Run inside the ros_container:
    docker exec -it ros_container bash
    source /opt/ros/noetic/setup.bash
    python3 /root/NavWareSet_parse/extract_static_map.py --scene 1
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag

sys.path.insert(0, str(Path(__file__).resolve().parent))
from socnav_lib.paths import ExperimentPaths
from socnav_lib.robot_config import detect_robot_type, load_robot_configs


def load_static_grid(bag_path: Path, robot_type_override: str = None):
    bag = rosbag.Bag(str(bag_path))
    try:
        topics = bag.get_type_and_topic_info().topics.keys()
        robot_type = robot_type_override or detect_robot_type(topics)
        msg = None
        for _, m, _ in bag.read_messages(topics=["/static_obstacle_ros_map"]):
            msg = m
            break
    finally:
        bag.close()
    if msg is None:
        raise RuntimeError(f"No /static_obstacle_ros_map message found in {bag_path}")
    return msg, robot_type


def occupancy_grid_to_array(msg) -> np.ndarray:
    data = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
    grid = np.zeros_like(data, dtype=np.int8)
    grid[data < 0] = -1
    grid[data >= 50] = 1
    return grid


def extract_walls(grid: np.ndarray, resolution: float, x_orig: float, y_orig: float, epsilon_px: float):
    occupied = (grid == 1).astype(np.uint8) * 255
    contours, _ = cv2.findContours(occupied, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    walls = []
    for contour in contours:
        if cv2.contourArea(contour) < 4:
            continue
        approx = cv2.approxPolyDP(contour, epsilon_px, True).reshape(-1, 2)
        if len(approx) < 2:
            continue
        for i in range(len(approx)):
            col1, row1 = approx[i]
            col2, row2 = approx[(i + 1) % len(approx)]
            x1, y1 = x_orig + col1 * resolution, y_orig + row1 * resolution
            x2, y2 = x_orig + col2 * resolution, y_orig + row2 * resolution
            walls.append([float(x1), float(y1), float(x2), float(y2)])
    return walls


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument("--experiments-root", default="/root/experiments")
    parser.add_argument("--wall-epsilon-px", type=float, default=2.0,
                         help="cv2.approxPolyDP simplification tolerance, in grid cells")
    parser.add_argument("--robot-type", choices=sorted(load_robot_configs()), default=None,
                         help="skip topic-based detection and force this robot type")
    args = parser.parse_args()

    paths = ExperimentPaths(args.scene, Path(args.experiments_root))
    msg, robot_type = load_static_grid(paths.robot_bag, args.robot_type)
    grid = occupancy_grid_to_array(msg)

    info = msg.info
    q = info.origin.orientation
    angle_orig = 2.0 * np.arctan2(q.z, q.w)

    walls = extract_walls(grid, info.resolution, info.origin.position.x, info.origin.position.y,
                           args.wall_epsilon_px)

    output = {
        "robot_type": robot_type,
        "grid": {
            "width": int(info.width),
            "height": int(info.height),
            "cell_size": float(info.resolution),
            "x_orig": float(info.origin.position.x),
            "y_orig": float(info.origin.position.y),
            "angle_orig": float(angle_orig),
            "data": grid.tolist(),
        },
        "walls": walls,
    }

    out_path = paths.static_map_json
    with open(out_path, "w") as f:
        json.dump(output, f)
    print(f"Wrote {out_path} ({len(walls)} wall segments, robot_type={robot_type})")


if __name__ == "__main__":
    main()
