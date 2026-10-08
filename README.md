# dexterity-ros2

ROS 2 Jazzy workspace (`src/`) for closed-loop hand-eye manipulation on a simulated **Universal Robots
UR5e** arm with a Robotiq 2F-85 gripper, hazmat sign detection and 6D object pose tracking (ICG / M3T).
It is a ROS 2 port of the GET Lab `dexterity` / Object Handling code.

## Contents

| Folder | Description |
|---|---|
| [`dexterity_ros2/`](dexterity_ros2/README.md) | Hand-eye controller (`dexterity`), messages, rqt GUI panel, evaluation and the UR5e Gazebo simulation (`dexterity_ur`) |
| `pose_estimation_icg/` | Object pose tracking with ICG (see [`ICG_README.md`](pose_estimation_icg/ICG_README.md)) |
| `pose_estimation_m3t/` | Object pose tracking with M3T (see [`M3T_README.md`](pose_estimation_m3t/M3T_README.md)) |
| `hazmat_detection/` | Hazmat sign detection by SIFT template matching with Hough voting |
| `dependencies.repos` | Third-party gripper repositories (`ros2_robotiq_gripper`, `serial`) with tested branches |

`grippers/` is **not** part of this repository (git ignored). It is cloned from `dependencies.repos`.

## Requirements

- Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic
- UR packages (`ur_description`, `ur_simulation_gz`, `ur_moveit_config`), MoveIt 2
- `libglfw3-dev` (ICG / M3T trackers), `python3-vcstool`

```bash
sudo apt install ros-jazzy-ur ros-jazzy-ur-simulation-gz ros-jazzy-moveit libglfw3-dev python3-vcstool
```

## Setup

```bash
git clone https://github.com/ashiqlathief/dexterity-ros2.git ~/ros2_ws/src
cd ~/ros2_ws
vcs import src < src/dependencies.repos      # clones ros2_robotiq_gripper, serial
```

## Build

Conda must be inactive, otherwise the message build uses conda's Python and fails with `No module named 'em'`.

```bash
conda deactivate
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional: system dependencies
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Build in the directory where the workspace lives. `build/` and `install/` store absolute paths, so copying them to
another location breaks them. Delete `build install log` and rebuild instead.

## Run

```bash
ros2 launch dexterity_ur ur_sim.launch.py
```

See [`dexterity_ros2/README.md`](dexterity_ros2/README.md) for the quick start, pipestar
tracking with M3T / ICG and the evaluation tools, and [`dexterity_ur/README.md`](dexterity_ros2/dexterity_ur/README.md)
for the UR5e simulation and launch arguments.

