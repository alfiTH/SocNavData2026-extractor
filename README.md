# Notes
## Install nvidia container
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
## If you have docker destop
docker context use default

realiza de nuevo el docker compose up

## If rviz fails

run this on host, not docker
xhost +local:

## Run rosbag
rosbag play --clock 1_robot.bag

## sync time sim
rosparam set use_sim_time true

## Checklist

- r = rosbag
- d = dataset
- c = calculate


| **Type**       | **Variable**            | **Description** |
|----------------|-------------------------|-----------------|
| **Robot**      | [r] Pose                    | 2D position (m) and orientation (rad) on the plane. |
|                | [r] Speed                   | Lineal (m/s), angular (rad/s). |
|                | [d] Drive                   | Categorical (differential / omni / ackerman). |
|                | [d] Shape                   | 2D *circle* (radius), *rectangle* (width, height), or *polygon* (list of points). |
| **Task**       | [idk] Type                    | Task type, either *“go-to”*, *“guide-to”* or *“follow”* $^*$. |
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
|                | [c] Area semantics          | Free text describing the area, e.g., “indoor”, “outdoor”, “kitchen”, “a science museum”. |
