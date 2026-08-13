# SocNavData2026 Extractor

Tools to turn raw ROS bag recordings into SocNavData2026-formatted trajectories.

## Setup

### Install the NVIDIA container toolkit

```bash
sudo apt-get update && sudo apt-get install -y --no-install-recommends \
   ca-certificates \
   curl \
   gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update

sudo apt install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### If you're using Docker Desktop

```bash
docker context use default
```

then relaunch `docker compose up`.

### If RViz fails to open

Run this on the host, not inside the container:

```bash
xhost +local:
```

### Play a rosbag

```bash
rosbag play --clock 1_robot.bag
```

### Sync simulated time

```bash
rosparam set use_sim_time true
```

## Data schema checklist

- **r** = derived from the rosbag
- **d** = fixed dataset-level default
- **c** = calculated from the extraction pipeline

| **Type**       | **Variable**            | **Description** |
|----------------|-------------------------|-----------------|
| **Robot**      | [r] Pose                    | 2D position (m) and orientation (rad) on the plane. |
|                | [r] Speed                   | Linear (m/s), angular (rad/s). |
|                | [d] Drive                   | Categorical (differential / omni / ackerman). |
|                | [d] Shape                   | 2D *circle* (radius), *rectangle* (width, height), or *polygon* (list of points). |
| **Task**       | [idk] Type                    | Task type, either *"go-to"*, *"guide-to"* or *"follow"* $^*$. |
|                | [idk] Position + threshold    | For go-to and guide-to tasks. 2D position + threshold (m). |
|                | [idk] Orientation + threshold | For go-to and guide-to tasks. Orientation + threshold (rad). |
|                | [idk] Human identifier        | For *guide-to* and *follow* tasks. |
|                | [idk] Context                 | Textual description of the context, in English. |
| **Humans**     | [r] Identifier              | Integer that uniquely identifies the human in a given episode. |
|                | [r] Pose                    | 2D position (m) and orientation (rad) on the plane. |
|                | [c] Full Pose (optional)    | The 3D position of the COCO-18 key point set. |
| **Objects**    | [c] Identifier              | Integer that uniquely identifies the object in a given episode. |
|                | [c] Type                    | Free text describing the type of the object. |
|                | [c] Pose                    | 2D position (m) and orientation (rad) on the plane. |
|                | [c] Shape                   | The 2D shape of the object. |
| **Environment**| [c] Walls                   | Sequence of polylines (2D, m). |
|                | [r] Grid                    | Occupancy map: 2D grid + resolution (m/cell). |
|                | [c] Area semantics          | Free text describing the area, e.g., "indoor", "outdoor", "kitchen", "a science museum". |

## Pipeline: from a ROS bag to a SocNavData2026 trajectory

All the extraction/pipeline scripts live in [`NavWareSet_parse/`](NavWareSet_parse/), at the repo root — separate from the `ros_parse` catkin package under `workspace/src/`, which only holds the live ROS node (`ros_parse_node.py`) used for online visualization, not this offline pipeline. The `docker-compose.yml` mounts `./NavWareSet_parse` into the container at `/root/NavWareSet_parse`, so the same folder is reachable from both sides. The process has two stages: one that needs ROS (runs inside `ros_container`) and another in pure Python (runs on the host, no Docker required).

### Stage 1 — extract the static map (needs ROS)

Reads the single `/static_obstacle_ros_map` message from the scene's `<n>_robot.bag`, detects the robot type (HSR/Jackal) from the topics present, and computes the walls by contour extraction over the occupied cells. Writes `experiments/<n>/<n>_static_map.json`.

```bash
docker exec -it ros_container bash
source /opt/ros/noetic/setup.bash
python3 /root/NavWareSet_parse/extract_static_map.py --scene 1
```

### Stage 2 — build the trajectory/trajectories (pure Python, no container)

Combines the grid/walls from stage 1 with the robot pose (`<n>_robot_pose.csv`), the per-frame 3D human bounding boxes (`scene_<n>/ann/*.pcd.json`) transformed from the GRS frame to the robot frame, and `key_id_map.json` for stable per-person IDs. Writes the result to `dataset/unlabeled/ros_extractor/`, following the exact format of the SocNavData2026 repo's `schema.json`.

```bash
python3 NavWareSet_parse/build_trajectory.py --scene 1
```

**Multiple runs per experiment**: a single scene can contain several "go-to" runs recorded back-to-back. Two cut patterns between one run and the next are detected automatically, and one file is generated per run (`1_0.json`, `1_1.json`, ...):

- **Long stop**: the robot comes to a complete stop for a while.
- **In-place turn**: the robot doesn't fully stop — it slows its forward speed, turns ~180° with almost no displacement (to face the next goal), and resumes moving forward. This is the common case in this capture (scene 1 has 9 turns of this kind, one every ~26s → 10 runs). Linear speed alone isn't reliable for detecting this: deriving pose by finite differences amplifies the pose's own noise into ±0.1–0.2 m/s spikes even while the robot is only turning, so this case is detected by orientation change (≥120° over an 8s window) with minimal displacement (≤1m), not by speed.

If a scene is segmented incorrectly (merges two runs, or splits one into several), adjust the thresholds:

```bash
python3 NavWareSet_parse/build_trajectory.py --scene 1 \
  --stop-speed-threshold 0.05 --min-stop-duration 3.0 --min-run-duration 2.0 \
  --turnaround-angle-deg 120 --turnaround-window 1.75 --turnaround-max-displacement 1.0
# or, to disable segmentation and keep a single file:
python3 NavWareSet_parse/build_trajectory.py --scene 1 --no-split
```

`metadata` (the textual context) and the `goal` thresholds are left as default/placeholder values — they can't be recovered from the ROS data, so they need to be filled in by hand per scene.

### Validate against the official schema

```bash
cd /home/alfith/Code/SocNavData2026/dataset/check_trajectory_format
python3 checkjson.py /home/alfith/Code/SocNavData2026-extractor/dataset/unlabeled/ros_extractor/1_0.json
```

### Visualize the result

`visualize_trajectory.py` plays back, in 2D (top-down view), the grid, walls, robot, and people from one or more already-generated trajectory files. It doesn't need ROS, only numpy/cv2.

```bash
# load every run of scene 1 (1_0.json, 1_1.json, ...)
python3 NavWareSet_parse/visualize_trajectory.py --scene 1

# or a specific file
python3 NavWareSet_parse/visualize_trajectory.py --file dataset/unlabeled/ros_extractor/1_0.json
```

Controls: `space` pause/resume, `←`/`→` step one frame back/forward, `↑`/`↓` change playback speed, `n`/`p` next/previous run, `s` save the current frame as PNG, `q`/`Esc` quit.

### Discarded frames (no nearby robot pose)

`build_trajectory.py` discards an annotated frame if it can't find any `<n>_robot_pose.csv` sample within 150 ms (`MAX_POSE_MATCH_GAP_NS`). Instead of silently disappearing, those frames are saved to `<n>_skipped_frames.json` (approximate robot pose + people at that instant) so they can be reviewed:

```bash
python3 NavWareSet_parse/visualize_trajectory.py --skipped 1
```

Same controls as normal mode (no goal or trace, since these frames aren't part of any run). The header shows the time gap (`gap=...ms`) and the source `.pcd.json` — if the gap is small and the pose looks correct, you can raise `MAX_POSE_MATCH_GAP_NS` in `build_trajectory.py`; if the robot is clearly out of place, it's better to leave it out.

### Robot type and footprint (config persisted across scenes)

`extract_static_map.py` detects HSR or Jackal automatically from the topics in `<n>_robot.bag` (`NavWareSet_parse/socnav_lib/robot_config.py::detect_robot_type`) and stores it as `robot_type` in `<n>_static_map.json`. If detection fails, or you want to force it, pass it manually:

```bash
python3 /root/NavWareSet_parse/extract_static_map.py --scene 1 --robot-type hsr
```

The footprint (`shape`) and drive type (`drive`) of each platform are **not** saved per scene: they live in `NavWareSet_parse/socnav_lib/robot_configs.json`, a shared file used the same way across all scenes. Edit it directly if you have exact HSR/Jackal measurements, or to add a new platform — no need to touch the Python code.

### Editing walls by hand

The walls that `extract_static_map.py` computes from the grid contour pick up anything marked as occupied (furniture included), not just the room's real walls. `edit_walls.py` is a simple interactive editor to clean them up:

```bash
python3 NavWareSet_parse/edit_walls.py --scene 1
```

Starting point, in this order: the scene's own override if it already exists (to keep refining a previous edit); otherwise the override from the nearest earlier scene (by default it looks for scenes with a lower number — useful because consecutive scenes are usually recorded in the same room, saving you from redrawing; disable this with `--no-previous-scene` or set a specific base scene with `--base-scene N`); if none exists, the auto-extracted walls for this scene (always visible in the background, in light gray, as a reference). `--from-scratch` skips all of this and starts blank.

```bash
# force scene 2 as the starting point, without auto-searching
python3 NavWareSet_parse/edit_walls.py --scene 5 --base-scene 2
```

Controls: left click adds a point, right click/`n` closes the current polyline and starts a new one, `z` undo, `r` reset to the auto-extracted walls (for this scene), `c` start from scratch, `g` toggle the reference on/off, `s` save, `q`/`Esc` quit.

If `<n>_walls_override.json` exists, `build_trajectory.py` uses it instead of the auto-extracted walls for every run of that scene. Unlike the rest of `experiments/`, these files **are** tracked in git (an exception in `.gitignore`) because they're manual work, not a regenerable artifact.

### New scenes (2, 3, ...)

If a scene only has the `.zip` files (not yet unpacked), you don't need to unzip them by hand: `ExperimentPaths` (in `NavWareSet_parse/socnav_lib/paths.py`) unpacks `0N_annotated.zip`, `0N_poses.zip` and `0N_robot.zip` automatically the first time they're needed.

## Useful commands — full sequential process

Quick reference for running the whole pipeline for a new scene `N`, start to finish:

```bash
# 1. Extract the static map (inside the ROS container)
docker exec -it ros_container bash
source /opt/ros/noetic/setup.bash
python3 /root/NavWareSet_parse/extract_static_map.py --scene N
exit

# 2. (optional) Clean up the auto-extracted walls by hand (on the host)
python3 NavWareSet_parse/edit_walls.py --scene N

# 3. Build the trajectory/trajectories (on the host, pure Python)
python3 NavWareSet_parse/build_trajectory.py --scene N

# 4. Visualize the result to sanity-check it
python3 NavWareSet_parse/visualize_trajectory.py --scene N

# 5. Review any frames that were discarded for lacking a nearby robot pose
python3 NavWareSet_parse/visualize_trajectory.py --skipped N

# 6. Validate every generated file against the official schema
cd ~/Code/SocNavData2026/dataset/check_trajectory_format
python3 checkjson.py ~/Code/SocNavData2026-extractor/dataset/unlabeled/ros_extractor/N_0.json
```

Notes:
- Step 1 needs ROS/Docker; steps 2–6 run on the host with plain Python (numpy/cv2), no container required.
- Repeat step 6 for each generated run file (`N_0.json`, `N_1.json`, ...).
- `metadata` and the `goal` thresholds are placeholders after step 3 and must be filled in by hand per scene before the dataset is considered complete.
