#!/usr/bin/env python3
"""Stage 2 (no ROS needed): combines the static map from extract_static_map.py
with the pre-extracted annotated/poses data for a scene into a
SocNavData2026 schema.json-compliant trajectory JSON.

    metadata (context)   -> left as "" (SocNavData2026-extractor convention:
                             fill this in by hand, it isn't recoverable from ROS)
    grid, walls           -> <scene>_static_map.json (stage 1)
    robot.shape/drive      -> socnav_lib.robot_config, keyed by detected robot type
    robot.x/y/angle         -> <scene>_robot_pose.csv (already in the robot/map frame)
    robot.speed_*          -> finite differences of the pose CSV, rotated into the
                             robot's body frame
    goal                    -> heuristic: "go-to" the robot's final pose in the
                             sequence, with default thresholds (not present in the
                             ROS data either)
    people[].id/x/y          -> scene_<n>/ann/*.pcd.json cuboids (GRS frame),
                             mapped to stable ids via key_id_map.json and
                             transformed into the map frame via
                             <scene>_grs_to_bot_offset.json
    people[].angle           -> heading between consecutive positions of the same
                             id (cuboids carry no orientation)
    objects                  -> [] (no furniture/object annotations in this dataset)

Annotation frames whose nearest robot-pose sample is more than
MAX_POSE_MATCH_GAP_NS away get dropped rather than guessed at, and are
logged to <scene>_skipped_frames.json (people + approximate robot pose)
instead of silently vanishing -- inspect them with
`visualize_trajectory.py --skipped <scene>` to check whether dropping them
was actually the right call.

A single recorded scene can contain several distinct "go-to" runs back to
back. Two patterns mark the boundary between one run and the next, and both
are auto-detected:
  - a sustained low-speed pause (the robot fully stops for a while), or
  - a near-in-place ~180 degree turn (the robot's forward speed drops to
    ~0, it spins to face the next goal, then moves off again -- no long
    pause in between). Linear speed alone is too noisy to catch this one
    reliably (finite-differencing the pose CSV amplifies pose jitter into
    +/-0.1-0.2 m/s spikes even while the robot is only rotating), so this
    is detected from heading change vs. displacement instead: a large
    turn with very little translation.
Each run is written as its own SocNavData2026 trajectory file, named
<scene>_<run_index>.json (run_index starting at 0). Tune
--stop-speed-threshold/--min-stop-duration (full stops) and
--turnaround-angle-deg/--turnaround-window/--turnaround-max-displacement
(in-place turns) if runs get merged or split incorrectly for a given scene.

Run (no ROS/container needed, just pandas/numpy):
    python3 build_trajectory.py --scene 1
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from socnav_lib.defaults import default_experiments_root, default_out_root
from socnav_lib.paths import ExperimentPaths
from socnav_lib.robot_config import robot_config_for
from socnav_lib.transforms import GRS_TO_ROBOT_CLOCK_OFFSET_NS, GrsToMapTransform, wrap_angle

DEFAULT_POS_THRESHOLD = 0.3
DEFAULT_ANGLE_THRESHOLD = math.radians(20)
MAX_POSE_MATCH_GAP_NS = 150_000_000
MIN_PERSON_MOVE_M = 0.03
DEFAULT_STOP_SPEED_THRESHOLD = 0.05
DEFAULT_MIN_STOP_DURATION = 3.0
DEFAULT_MIN_RUN_DURATION = 5.0
DEFAULT_MIN_RUN_STEPS = 5
DEFAULT_TURNAROUND_ANGLE_DEG = 120.0
DEFAULT_TURNAROUND_WINDOW_S = 1.75
DEFAULT_TURNAROUND_MAX_DISPLACEMENT_M = 1.0


def parse_grs_timestamp_ns(pcd_json_path: Path) -> int:
    stem = pcd_json_path.name[: -len(".pcd.json")]
    sec_str, _, frac_str = stem.partition(".")
    frac_str = (frac_str + "000000000")[:9]
    return int(sec_str) * 1_000_000_000 + int(frac_str)


def nearest_index(sorted_values: np.ndarray, target: int):
    idx = np.searchsorted(sorted_values, target)
    if idx <= 0:
        return 0
    if idx >= len(sorted_values):
        return len(sorted_values) - 1
    before, after = sorted_values[idx - 1], sorted_values[idx]
    return idx - 1 if (target - before) <= (after - target) else idx


def load_robot_track(csv_path: Path):
    df = pd.read_csv(csv_path).sort_values("timestamp_ns").reset_index(drop=True)
    t = df["timestamp_ns"].to_numpy() / 1e9
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    yaw = np.unwrap(df["yaw_rad"].to_numpy())
    vx_world = np.gradient(x, t)
    vy_world = np.gradient(y, t)
    vyaw = np.gradient(yaw, t)
    return {
        "ts_ns": df["timestamp_ns"].to_numpy(),
        "x": x,
        "y": y,
        "yaw": df["yaw_rad"].to_numpy(),
        "vx_world": vx_world,
        "vy_world": vy_world,
        "vyaw": vyaw,
    }


def nearest_robot_gap(track, ts_ns: int):
    """Index of the closest robot-track sample to ts_ns, and how far off it
    is (ns) -- returned even when that gap is too large to use, so a
    rejected match can still be reported/inspected."""
    i = nearest_index(track["ts_ns"], ts_ns)
    gap = abs(int(track["ts_ns"][i]) - ts_ns)
    return i, gap


def robot_sample_from_index(track, i: int):
    yaw = track["yaw"][i]
    c, s = math.cos(yaw), math.sin(yaw)
    vx_world, vy_world = track["vx_world"][i], track["vy_world"][i]
    speed_x = vx_world * c + vy_world * s
    speed_y = -vx_world * s + vy_world * c
    return {
        "x": float(track["x"][i]),
        "y": float(track["y"][i]),
        "angle": float(yaw),
        "speed_x": float(speed_x),
        "speed_y": float(speed_y),
        "speed_a": float(track["vyaw"][i]),
    }


def robot_sample_at(track, ts_ns: int):
    i, gap = nearest_robot_gap(track, ts_ns)
    if gap > MAX_POSE_MATCH_GAP_NS:
        return None
    return robot_sample_from_index(track, i)


def build_people(ann_frame: dict, key_id_map: dict, transform: GrsToMapTransform, prev_state: dict):
    people = []
    for fig in ann_frame.get("figures", []):
        object_key = fig["objectKey"]
        person_id = key_id_map.get(object_key)
        if person_id is None:
            continue
        pos = fig["geometry"]["position"]
        mx, my = transform.point(pos["x"], pos["y"])

        angle = prev_state.get(person_id, {}).get("angle", 0.0)
        prev = prev_state.get(person_id)
        if prev is not None:
            dx, dy = mx - prev["x"], my - prev["y"]
            if math.hypot(dx, dy) >= MIN_PERSON_MOVE_M:
                angle = math.atan2(dy, dx)
        prev_state[person_id] = {"x": mx, "y": my, "angle": angle}

        people.append({"id": int(person_id), "x": mx, "y": my, "angle": angle})
    return people


def mask_to_runs(mask: np.ndarray):
    n = len(mask)
    runs = []
    start = 0
    for i in range(1, n + 1):
        if i == n or mask[i] != mask[start]:
            runs.append((start, i, bool(mask[start])))
            start = i
    return runs


def find_stationary_mask(speed: list, ts: list, stop_speed_threshold: float) -> np.ndarray:
    # A rolling *median* (not mean) is needed here: finite-differencing the
    # pose CSV amplifies small position jitter into occasional +/-0.1-0.2 m/s
    # speed spikes even while the robot is essentially still, and those
    # spikes survive a boxcar mean but get rejected by a median.
    median_dt = float(np.median(np.diff(ts))) if len(ts) > 1 else 1.0
    window = max(1, round(1.0 / median_dt)) if median_dt > 0 else 1
    smoothed = pd.Series(speed).rolling(window, center=True, min_periods=1).median().to_numpy()
    return smoothed < stop_speed_threshold


def find_turnaround_mask(sequence, angle_threshold_rad: float, window_s: float, max_displacement_m: float) -> np.ndarray:
    """Flags spans where the robot turns by >= angle_threshold_rad within
    window_s seconds while barely translating (<= max_displacement_m):
    the "stop, spin ~180 deg to face the next goal, move off again"
    pattern, which a speed-only check can miss (see find_stationary_mask)."""
    ts = np.array([s["timestamp"] for s in sequence])
    x = np.array([s["robot"]["x"] for s in sequence])
    y = np.array([s["robot"]["y"] for s in sequence])
    angle = np.unwrap([s["robot"]["angle"] for s in sequence])
    n = len(sequence)
    mask = np.zeros(n, dtype=bool)
    j = 0
    for i in range(n):
        j = max(j, i)
        while j < n - 1 and ts[j] - ts[i] < window_s:
            j += 1
        if j <= i:
            continue
        dtheta = abs(angle[j] - angle[i])
        disp = math.hypot(x[j] - x[i], y[j] - y[i])
        if dtheta >= angle_threshold_rad and disp <= max_displacement_m:
            mask[i:j + 1] = True
    return mask


def split_into_runs(sequence, stop_speed_threshold, min_stop_duration, min_run_duration, min_run_steps,
                     turnaround_angle_deg, turnaround_window_s, turnaround_max_displacement_m):
    """Splits one continuous recorded sequence into separate go-to runs, at
    sustained low-speed pauses and/or near-in-place turnarounds (see
    find_stationary_mask / find_turnaround_mask)."""
    n = len(sequence)
    ts = [step["timestamp"] for step in sequence]
    speed = [math.hypot(step["robot"]["speed_x"], step["robot"]["speed_y"]) for step in sequence]

    stationary = find_stationary_mask(speed, ts, stop_speed_threshold)
    turning = find_turnaround_mask(sequence, math.radians(turnaround_angle_deg), turnaround_window_s,
                                    turnaround_max_displacement_m)

    long_pauses = [(a, b) for (a, b, is_stat) in mask_to_runs(stationary)
                   if is_stat and (ts[b - 1] - ts[a]) >= min_stop_duration]
    turn_pauses = [(a, b) for (a, b, is_turn) in mask_to_runs(turning) if is_turn]

    cuts = []
    for a, b in sorted(long_pauses + turn_pauses):
        if cuts and a <= cuts[-1][1]:
            cuts[-1] = (cuts[-1][0], max(cuts[-1][1], b))
        else:
            cuts.append((a, b))

    segments = []
    cursor = 0
    for a, b in cuts:
        if a > cursor:
            segments.append((cursor, a))
        cursor = b
    if cursor < n:
        segments.append((cursor, n))

    return [(a, b) for (a, b) in segments
            if (b - a) >= min_run_steps and (ts[b - 1] - ts[a]) >= min_run_duration]


def finalize_trajectory(sequence, static_map, walls):
    last_robot = sequence[-1]["robot"]
    goal = {
        "type": "go-to",
        "human": None,
        "x": last_robot["x"],
        "y": last_robot["y"],
        "angle": last_robot["angle"],
        "pos_threshold": DEFAULT_POS_THRESHOLD,
        "angle_threshold": DEFAULT_ANGLE_THRESHOLD,
    }
    for step in sequence:
        step["goal"] = goal
    return {
        "metadata": "",
        "grid": static_map["grid"],
        "walls": walls,
        "sequence": sequence,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument("--experiments-root", default=None)
    parser.add_argument("--out-root", default=None)
    parser.add_argument("--stop-speed-threshold", type=float, default=DEFAULT_STOP_SPEED_THRESHOLD,
                         help="m/s below which the robot is considered stopped (default: %(default)s)")
    parser.add_argument("--min-stop-duration", type=float, default=DEFAULT_MIN_STOP_DURATION,
                         help="seconds of sustained stop that splits two runs (default: %(default)s)")
    parser.add_argument("--min-run-duration", type=float, default=DEFAULT_MIN_RUN_DURATION,
                         help="runs shorter than this (seconds) are discarded (default: %(default)s)")
    parser.add_argument("--turnaround-angle-deg", type=float, default=DEFAULT_TURNAROUND_ANGLE_DEG,
                         help="heading change that counts as a turnaround between runs (default: %(default)s)")
    parser.add_argument("--turnaround-window", type=float, default=DEFAULT_TURNAROUND_WINDOW_S,
                         help="seconds within which that heading change must happen (default: %(default)s)")
    parser.add_argument("--turnaround-max-displacement", type=float,
                         default=DEFAULT_TURNAROUND_MAX_DISPLACEMENT_M,
                         help="max meters travelled during a turnaround for it to still count as "
                              "in-place (default: %(default)s)")
    parser.add_argument("--no-split", action="store_true",
                         help="write the whole scene as a single run, ignoring pauses/turnarounds")
    args = parser.parse_args()

    experiments_root = Path(args.experiments_root) if args.experiments_root else default_experiments_root()
    out_root = Path(args.out_root) if args.out_root else default_out_root()

    paths = ExperimentPaths(args.scene, experiments_root)

    if not paths.static_map_json.is_file():
        sys.exit(f"Missing {paths.static_map_json}. Run extract_static_map.py --scene {args.scene} first "
                  f"(inside ros_container, it needs rosbag).")
    static_map = json.loads(paths.static_map_json.read_text())
    if paths.walls_override_json.is_file():
        walls = json.loads(paths.walls_override_json.read_text())
        print(f"Using manually-edited walls from {paths.walls_override_json} "
              f"({len(walls)} segments, instead of the {len(static_map['walls'])} auto-extracted ones)")
    else:
        walls = static_map["walls"]

    robot_config = robot_config_for(static_map["robot_type"])
    robot_track = load_robot_track(paths.robot_pose_csv)
    offset = json.loads(paths.grs_to_bot_offset_json.read_text())
    transform = GrsToMapTransform(offset)
    key_id_map = json.loads(paths.key_id_map_json.read_text())["objects"]

    ann_files = sorted(paths.ann_dir.glob("*.pcd.json"), key=parse_grs_timestamp_ns)
    if not ann_files:
        sys.exit(f"No annotation frames found in {paths.ann_dir}")

    sequence = []
    skipped_frames = []
    prev_person_state = {}
    for ann_path in ann_files:
        grs_ns = parse_grs_timestamp_ns(ann_path)
        robot_ns = grs_ns + GRS_TO_ROBOT_CLOCK_OFFSET_NS

        idx, gap_ns = nearest_robot_gap(robot_track, robot_ns)
        ann_frame = json.loads(ann_path.read_text())
        people = build_people(ann_frame, key_id_map, transform, prev_person_state)

        if gap_ns > MAX_POSE_MATCH_GAP_NS:
            skipped_frames.append({
                "ann_file": ann_path.name,
                "timestamp": robot_ns / 1e9,
                "gap_ms": gap_ns / 1e6,
                "robot": {"shape": robot_config["shape"], **robot_sample_from_index(robot_track, idx)},
                "people": people,
            })
            continue

        sequence.append({
            "timestamp": robot_ns / 1e9,
            "robot": {"shape": robot_config["shape"], **robot_sample_from_index(robot_track, idx)},
            "people": people,
            "objects": [],
        })

    if skipped_frames:
        paths.skipped_frames_json.write_text(json.dumps({
            "scene": args.scene,
            "walls": walls,
            "frames": skipped_frames,
        }))
        print(f"Wrote {paths.skipped_frames_json} ({len(skipped_frames)} skipped frames, "
              f"gap_ms range {min(f['gap_ms'] for f in skipped_frames):.0f}-"
              f"{max(f['gap_ms'] for f in skipped_frames):.0f} -- inspect with "
              f"visualize_trajectory.py --skipped {args.scene})")

    if not sequence:
        sys.exit("No sequence steps could be built (robot pose track never matched an annotation frame).")

    if args.no_split:
        runs = [(0, len(sequence))]
    else:
        runs = split_into_runs(sequence, args.stop_speed_threshold, args.min_stop_duration,
                                args.min_run_duration, DEFAULT_MIN_RUN_STEPS,
                                args.turnaround_angle_deg, args.turnaround_window,
                                args.turnaround_max_displacement)
        if not runs:
            sys.exit("No run survived segmentation -- loosen --stop-speed-threshold/--min-stop-duration/"
                      "--min-run-duration/--turnaround-*, or pass --no-split.")

    out_root.mkdir(parents=True, exist_ok=True)
    for run_index, (a, b) in enumerate(runs):
        trajectory = finalize_trajectory(sequence[a:b], static_map, walls)
        out_path = out_root / f"{args.scene}_{run_index}.json"
        out_path.write_text(json.dumps(trajectory))
        print(f"Wrote {out_path}: {b - a} steps, "
              f"{trajectory['sequence'][0]['timestamp']:.1f}s -> {trajectory['sequence'][-1]['timestamp']:.1f}s")

    print(f"{len(runs)} run(s) written from {len(sequence)} matched steps "
          f"({len(skipped_frames)} annotation frames skipped, no matching robot pose), "
          f"drive={robot_config['drive']}.")
    print("Reminder: 'metadata' (context) and the goal thresholds are placeholders -- "
          "fill them in by hand, then validate with SocNavData2026/dataset/check_trajectory_format/checkjson.py")


if __name__ == "__main__":
    main()
