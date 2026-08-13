"""Per-platform metadata that has no ROS topic to read it from (footprint
shape, drive type). Kept in robot_configs.json, next to this file, rather
than hardcoded here -- it's the one thing meant to stay the same across
scenes, so it's a plain editable file instead of a per-experiment artifact.
Edit it directly if you have exact spec sheets for the HSR/Jackal, or to
add a new platform.
"""

import json
from pathlib import Path

ROBOT_CONFIGS_JSON = Path(__file__).resolve().parent / "robot_configs.json"


def load_robot_configs() -> dict:
    return json.loads(ROBOT_CONFIGS_JSON.read_text())


def robot_config_for(robot_type: str) -> dict:
    configs = load_robot_configs()
    if robot_type not in configs:
        raise KeyError(f"No entry for robot_type={robot_type!r} in {ROBOT_CONFIGS_JSON} "
                        f"(known: {sorted(configs)})")
    return configs[robot_type]


def detect_robot_type(topics) -> str:
    topics = list(topics)
    if any(t.startswith("/hsrb/") for t in topics):
        return "hsr"
    if "/amcl_pose" in topics or "/cmd_vel" in topics:
        return "jackal"
    raise ValueError(f"Could not detect robot type from topics: {sorted(topics)}")
