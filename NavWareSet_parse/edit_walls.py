#!/usr/bin/env python3
"""Simple interactive editor for a scene's walls. The walls that
build_trajectory.py writes into the final JSON come from contour tracing
on the occupancy grid (extract_static_map.py) -- that picks up every bit
of clutter the grid marks as occupied, not just the room's walls. This
tool lets you clean that up by hand: click out the polylines you actually
want, save, and build_trajectory.py will use them instead.

    python3 edit_walls.py --scene 1

Starting point, in order: this scene's own <scene>_walls_override.json if
one already exists (to keep refining a previous edit); otherwise the
closest lower-numbered scene's override, if any (rooms are usually reused
across consecutive scenes, so this saves re-drawing the same walls --
disable with --no-previous-scene or pick one explicitly with
--base-scene N); otherwise the auto-extracted walls from
<scene>_static_map.json. Pass --from-scratch to skip all of that and start
with a blank canvas. The auto-extracted walls for *this* scene are always
kept faintly visible in the background as a reference (toggle with 'g').

Controls:
    left click     add a point to the current polyline
    right click / n   finish the current polyline, start a new one
    z              undo the last point (or the last finished polyline)
    r              reset to the auto-extracted walls
    c              clear everything, start from a blank canvas
    g              toggle the faint auto-extracted-walls background
    s              save to <scene>_walls_override.json
    q / ESC        quit (prompts if there are unsaved changes)
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from socnav_lib.defaults import default_experiments_root
from socnav_lib.paths import ExperimentPaths
from socnav_lib.view import View

WINDOW_NAME = "SocNavData2026-extractor wall editor"
REFERENCE_COLOR = (210, 210, 210)
WALL_COLOR = (40, 40, 40)
POINT_COLOR = (0, 140, 255)
PREVIEW_COLOR = (0, 200, 0)


def polylines_to_segments(polylines):
    segments = []
    for pts in polylines:
        for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
            segments.append([x1, y1, x2, y2])
    return segments


def segments_to_polylines(segments):
    """Each saved segment becomes its own 2-point polyline. Good enough for
    re-editing (every segment stays individually selectable/undoable) even
    though a chain drawn as one polyline round-trips as several."""
    return [[(x1, y1), (x2, y2)] for x1, y1, x2, y2 in segments]


def find_previous_override(scene: int, experiments_root: Path):
    """Nearest lower-numbered scene that has its own walls override, if
    any -- consecutive scenes are usually recorded in the same room."""
    if not experiments_root.is_dir():
        return None
    older_scenes = sorted(
        (int(d.name) for d in experiments_root.iterdir() if d.is_dir() and d.name.isdigit() and int(d.name) < scene),
        reverse=True,
    )
    for n in older_scenes:
        candidate = experiments_root / str(n) / f"{n}_walls_override.json"
        if candidate.is_file():
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scene", type=int, required=True)
    parser.add_argument("--experiments-root", default=None)
    parser.add_argument("--from-scratch", action="store_true",
                         help="start with a blank canvas instead of the existing override/auto walls")
    parser.add_argument("--no-previous-scene", action="store_true",
                         help="don't fall back to a previous scene's override, go straight to auto-extracted walls")
    parser.add_argument("--base-scene", type=int, default=None,
                         help="start from this scene's override instead of auto-searching lower-numbered scenes")
    args = parser.parse_args()

    experiments_root = Path(args.experiments_root) if args.experiments_root else default_experiments_root()
    paths = ExperimentPaths(args.scene, experiments_root)

    if not paths.static_map_json.is_file():
        sys.exit(f"Missing {paths.static_map_json}. Run extract_static_map.py --scene {args.scene} first.")
    static_map = json.loads(paths.static_map_json.read_text())
    auto_walls = static_map["walls"]

    if args.from_scratch:
        polylines = []
    elif paths.walls_override_json.is_file():
        polylines = segments_to_polylines(json.loads(paths.walls_override_json.read_text()))
        print(f"Loaded {len(polylines)} segments from {paths.walls_override_json}")
    elif args.base_scene is not None:
        base_path = ExperimentPaths(args.base_scene, experiments_root).walls_override_json
        if not base_path.is_file():
            sys.exit(f"--base-scene {args.base_scene} has no {base_path.name}")
        polylines = segments_to_polylines(json.loads(base_path.read_text()))
        print(f"Starting from scene {args.base_scene}'s override ({len(polylines)} segments): {base_path}")
    else:
        previous = None if args.no_previous_scene else find_previous_override(args.scene, experiments_root)
        if previous is not None:
            polylines = segments_to_polylines(json.loads(previous.read_text()))
            print(f"Starting from a previous scene's override ({len(polylines)} segments): {previous}")
        else:
            polylines = segments_to_polylines(auto_walls)
            print(f"Starting from {len(polylines)} auto-extracted segments")

    xs = [x for x1, y1, x2, y2 in auto_walls for x in (x1, x2)]
    ys = [y for x1, y1, x2, y2 in auto_walls for y in (y1, y2)]
    view = View(xs, ys)

    state = {"current": [], "mouse": None, "show_ref": True, "dirty": False}

    def on_mouse(event, px, py, flags, userdata):
        state["mouse"] = (px, py)
        if event == cv2.EVENT_LBUTTONDOWN:
            state["current"].append(view.to_world(px, py))
            state["dirty"] = True
        elif event == cv2.EVENT_RBUTTONDOWN and state["current"]:
            if len(state["current"]) >= 2:
                polylines.append(state["current"])
            state["current"] = []

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)

    while True:
        canvas = np.full((view.canvas_h, view.canvas_w, 3), 255, dtype=np.uint8)
        if state["show_ref"]:
            for x1, y1, x2, y2 in auto_walls:
                cv2.line(canvas, view.to_px(x1, y1), view.to_px(x2, y2), REFERENCE_COLOR, 1)
        for pts in polylines:
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                cv2.line(canvas, view.to_px(x1, y1), view.to_px(x2, y2), WALL_COLOR, 2)
        for pt in state["current"]:
            cv2.circle(canvas, view.to_px(*pt), 3, POINT_COLOR, -1)
        for (x1, y1), (x2, y2) in zip(state["current"], state["current"][1:]):
            cv2.line(canvas, view.to_px(x1, y1), view.to_px(x2, y2), POINT_COLOR, 2)
        if state["current"] and state["mouse"]:
            cv2.line(canvas, view.to_px(*state["current"][-1]), state["mouse"], PREVIEW_COLOR, 1)

        status = f"{len(polylines)} polylines, {len(state['current'])} pts in progress"
        status += "  [unsaved]" if state["dirty"] else "  [saved]"
        cv2.putText(canvas, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imshow(WINDOW_NAME, canvas)

        key = cv2.waitKey(30) & 0xFF
        if key in (27, ord("q")):
            if state["dirty"]:
                print("You have unsaved changes -- press 's' to save first, or quit again to discard.")
                state["dirty"] = False
                continue
            break
        elif key == ord("n"):
            if len(state["current"]) >= 2:
                polylines.append(state["current"])
            state["current"] = []
        elif key == ord("z"):
            if state["current"]:
                state["current"].pop()
            elif polylines:
                polylines.pop()
            state["dirty"] = True
        elif key == ord("r"):
            polylines = segments_to_polylines(auto_walls)
            state["current"] = []
            state["dirty"] = True
        elif key == ord("c"):
            polylines = []
            state["current"] = []
            state["dirty"] = True
        elif key == ord("g"):
            state["show_ref"] = not state["show_ref"]
        elif key == ord("s"):
            segments = polylines_to_segments(polylines)
            paths.walls_override_json.write_text(json.dumps(segments))
            print(f"Saved {len(segments)} segments to {paths.walls_override_json}")
            state["dirty"] = False

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
