#!/usr/bin/env python3
"""Top-down player for the SocNavData2026-schema trajectory JSON files
produced by build_trajectory.py. No ROS needed, just numpy/cv2.

A scene can be split into several runs (1_0.json, 1_1.json, ...); pass
--scene to load all of them and step between with n/p, or --file to load
one specific trajectory (or several) directly. Pass --skipped SCENE
instead to step through that scene's <scene>_skipped_frames.json (the
annotation frames build_trajectory.py couldn't match to a robot pose and
dropped) -- same controls, minus goal/trail, plus the match gap in ms and
the source .pcd.json file per frame, so you can judge whether dropping
them was actually the right call.

    python3 visualize_trajectory.py --scene 1
    python3 visualize_trajectory.py --file ../dataset/unlabeled/ros_extractor/1_0.json
    python3 visualize_trajectory.py --skipped 1

Playback controls:
    space        play / pause
    right/left   step one frame forward / back
    up/down      speed up / slow down playback
    n/p          next / previous run (only relevant with --scene)
    s            save the current frame as a PNG next to the trajectory file
    q / ESC      quit
"""

import argparse
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from socnav_lib.defaults import default_experiments_root, default_out_root
from socnav_lib.paths import ExperimentPaths
from socnav_lib.view import View

WINDOW_NAME = "SocNavData2026-extractor trajectory viewer"
PERSON_COLORS = [
    (66, 133, 244), (219, 68, 55), (244, 180, 0), (15, 157, 88),
    (171, 71, 188), (255, 112, 67), (0, 172, 193), (158, 157, 36),
]


def view_from_trajectory(trajectory: dict) -> View:
    xs, ys = [], []
    for x1, y1, x2, y2 in trajectory["walls"]:
        xs += [x1, x2]
        ys += [y1, y2]
    for step in trajectory["sequence"]:
        xs.append(step["robot"]["x"])
        ys.append(step["robot"]["y"])
        for p in step["people"]:
            xs.append(p["x"])
            ys.append(p["y"])
    return View(xs, ys)


def draw_static_layer(trajectory: dict, view: View) -> np.ndarray:
    canvas = np.full((view.canvas_h, view.canvas_w, 3), 255, dtype=np.uint8)
    for x1, y1, x2, y2 in trajectory["walls"]:
        cv2.line(canvas, view.to_px(x1, y1), view.to_px(x2, y2), (60, 60, 60), 2)
    return canvas


def draw_robot(canvas, view: View, robot: dict):
    center = view.to_px(robot["x"], robot["y"])
    radius_m = max(robot["shape"].get("width", 0.4), robot["shape"].get("length", 0.4)) / 2.0
    radius_px = max(int(radius_m * view.scale), 4)
    cv2.circle(canvas, center, radius_px, (0, 140, 255), 2)
    tip = view.to_px(robot["x"] + math.cos(robot["angle"]) * radius_m,
                      robot["y"] + math.sin(robot["angle"]) * radius_m)
    cv2.line(canvas, center, tip, (0, 140, 255), 2)


def draw_goal(canvas, view: View, goal: dict):
    center = view.to_px(goal["x"], goal["y"])
    cv2.drawMarker(canvas, center, (0, 180, 0), markerType=cv2.MARKER_STAR, markerSize=16, thickness=2)
    radius_px = max(int(goal.get("pos_threshold", 0.3) * view.scale), 1)
    cv2.circle(canvas, center, radius_px, (0, 180, 0), 1)


def draw_person(canvas, view: View, person: dict):
    color = PERSON_COLORS[person["id"] % len(PERSON_COLORS)]
    center = view.to_px(person["x"], person["y"])
    cv2.circle(canvas, center, 6, color, -1)
    tip = view.to_px(person["x"] + math.cos(person["angle"]) * 0.3, person["y"] + math.sin(person["angle"]) * 0.3)
    cv2.line(canvas, center, tip, color, 2)
    cv2.putText(canvas, str(person["id"]), (center[0] + 8, center[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)


def draw_trail(canvas, view: View, sequence: list, upto_idx: int, max_len=200):
    start = max(0, upto_idx - max_len)
    pts = [view.to_px(step["robot"]["x"], step["robot"]["y"]) for step in sequence[start:upto_idx + 1]]
    for a, b in zip(pts, pts[1:]):
        cv2.line(canvas, a, b, (0, 140, 255), 1)


def render_frame(trajectory: dict, view: View, static_layer: np.ndarray, idx: int, label: str,
                  show_goal: bool = True, show_trail: bool = True) -> np.ndarray:
    canvas = static_layer.copy()
    sequence = trajectory["sequence"]
    step = sequence[idx]
    if show_goal and "goal" in step:
        draw_goal(canvas, view, step["goal"])
    if show_trail:
        draw_trail(canvas, view, sequence, idx)
    draw_robot(canvas, view, step["robot"])
    for person in step["people"]:
        draw_person(canvas, view, person)
    header = f"{label}  frame {idx + 1}/{len(sequence)}  t={step['timestamp']:.2f}s"
    if "gap_ms" in step:
        header += f"  gap={step['gap_ms']:.0f}ms  {step['ann_file']}"
    cv2.putText(canvas, header, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return canvas


def load_skipped(scene: int, experiments_root) -> dict:
    paths = ExperimentPaths(scene, experiments_root)
    if not paths.skipped_frames_json.is_file():
        sys.exit(f"No {paths.skipped_frames_json} -- either nothing was skipped, or "
                  f"build_trajectory.py --scene {scene} hasn't been run yet.")
    data = json.loads(paths.skipped_frames_json.read_text())
    return {"walls": data["walls"], "sequence": data["frames"]}


def resolve_files(args) -> list:
    if args.file:
        return [Path(f) for f in args.file]
    out_root = Path(args.out_root) if args.out_root else default_out_root()
    files = sorted(out_root.glob(f"{args.scene}_*.json"),
                    key=lambda p: int(p.stem.split("_")[-1]))
    if not files:
        sys.exit(f"No {args.scene}_*.json files found in {out_root}. Run build_trajectory.py --scene {args.scene} "
                  f"first, or pass --file explicitly.")
    return files


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=int, help="load every run of this scene from --out-root")
    parser.add_argument("--file", nargs="+", help="load one or more trajectory JSON files directly")
    parser.add_argument("--skipped", type=int, metavar="SCENE",
                         help="load that scene's <scene>_skipped_frames.json instead of a built trajectory")
    parser.add_argument("--experiments-root", default=None, help="only used with --skipped")
    parser.add_argument("--out-root", default=None)
    parser.add_argument("--fps", type=float, default=10.0, help="initial playback speed (default: %(default)s)")
    args = parser.parse_args()

    if sum(x is not None for x in (args.scene, args.file, args.skipped)) != 1:
        parser.error("pass exactly one of --scene, --file, --skipped")

    show_goal, show_trail = True, True
    if args.skipped is not None:
        experiments_root = Path(args.experiments_root) if args.experiments_root else default_experiments_root()
        trajectories = [load_skipped(args.skipped, experiments_root)]
        labels = [f"scene {args.skipped} skipped frames"]
        save_stems = [ExperimentPaths(args.skipped, experiments_root).skipped_frames_json]
        show_goal, show_trail = False, False
    else:
        files = resolve_files(args)
        trajectories = [json.loads(f.read_text()) for f in files]
        labels = [f.stem for f in files]
        save_stems = files

    views = [view_from_trajectory(t) for t in trajectories]
    static_layers = [draw_static_layer(t, v) for t, v in zip(trajectories, views)]

    run = 0
    idx = 0
    playing = True
    fps = args.fps

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    while True:
        trajectory, view, static_layer = trajectories[run], views[run], static_layers[run]
        idx = max(0, min(idx, len(trajectory["sequence"]) - 1))
        frame = render_frame(trajectory, view, static_layer, idx, labels[run],
                              show_goal=show_goal, show_trail=show_trail)
        cv2.imshow(WINDOW_NAME, frame)

        delay_ms = max(1, int(1000 / fps)) if playing else 30
        key = cv2.waitKey(delay_ms) & 0xFF

        if key in (27, ord("q")):
            break
        elif key == ord(" "):
            playing = not playing
        elif key == 83 or key == ord("d"):  # right arrow
            idx += 1
            playing = False
        elif key == 81 or key == ord("a"):  # left arrow
            idx -= 1
            playing = False
        elif key == 82:  # up arrow
            fps = min(fps * 1.5, 200)
        elif key == 84:  # down arrow
            fps = max(fps / 1.5, 0.5)
        elif key == ord("n"):
            run = (run + 1) % len(trajectories)
            idx = 0
        elif key == ord("p"):
            run = (run - 1) % len(trajectories)
            idx = 0
        elif key == ord("s"):
            out_png = save_stems[run].with_suffix(f".frame{idx:05d}.png")
            cv2.imwrite(str(out_png), frame)
            print(f"Saved {out_png}")
        elif playing:
            idx += 1
            if idx >= len(trajectory["sequence"]):
                idx = 0

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
