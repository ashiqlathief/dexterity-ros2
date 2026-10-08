# dexterity (ROS 2 Jazzy)

ROS 2 port of the GET Lab `dexterity` node (closed-loop hand-eye coordination, M.Sc. thesis
S. Charania, 2023), running on a simulated **UR5e** with a Robotiq 2F-85 gripper instead of the GETjag.

| Package | Contents |
|---|---|
| `dexterity_msgs` | `Dexterity.action`, `ArmHome.srv`, `EstimatedPose.msg` (pose input from ICG / M3T tracker nodes), `DexterityEvent.msg` (task and planning events on `/dexterity/events`) |
| `dexterity` | `dexterity_node` (action server, perception, controller) and `dexterity_client` |
| `dexterity_gui` | rqt panel "Dexterity Arm Control" (port of the `ArmControl` widget of `object_handling_gui`) |
| `dexterity_ur` | Launch file, parameters, robot stand and the AprilTag / pipestar targets for the UR5e simulation |
| `dexterity_evaluation` | Ground truth evaluation in Gazebo: tracking error, true gripper distance, planning times (port of the simulator mode of `closed_loop_touch`) |

## Setup and build

Workspace layout: `src/dexterity_ros2`, `src/pose_estimation_m3t`, `src/pose_estimation_icg` and
`src/hazmat_detection` are the ported object handling code. `src/grippers` holds the third-party gripper packages
`ros2_robotiq_gripper` and `serial` (needed by `robotiq_driver`).

The third-party packages are not part of this repository (`src/grippers` is git ignored); they are listed in
`src/dependencies.repos` (branch and tested commit of each). One-time setup:

```bash
sudo apt install libglfw3-dev python3-vcstool   # GLFW for the M3T / ICG trackers, vcs
cd ~/ros2_ws
vcs import src < src/dependencies.repos         # clones ros2_robotiq_gripper, serial
```

Build (close a running simulation first, it uses the old build). Conda must be inactive, otherwise the message
build uses conda's Python and fails with `No module named 'em'`:

```bash
conda deactivate
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Quick start

One terminal per line. In every new terminal first run `conda deactivate` and
`source ~/ros2_ws/install/setup.bash`.

| Terminal | Command | What it does |
|---|---|---|
| 1 | `ros2 launch dexterity_ur ur_sim.launch.py` | Gazebo + UR5e + MoveIt + dexterity + Arm Control panel (AprilTag cube); wait ~25 s until the arm is at "Home" |

### Pipestar with M3T or ICG instead of the AprilTag cube

```bash
ros2 launch dexterity_ur ur_sim.launch.py target:=asymmetric_pipestar_without_cap   # terminal 1

conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch pose_estimation_m3t m3t.launch.py     # extra terminal (or: ros2 launch pose_estimation_icg icg.launch.py)


cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --packages-select dexterity_gui
source install/setup.bash
rqt --standalone dexterity_gui/ArmControl
```

In the Arm Control panel:

1. Perception Parameters: tracker `m3t` (or `icg`), object model `asymmetric_pipestar_without_cap`.
2. Tracker tab: **Start Bootstrapping**, click the 6 points (**Show Reference** shows where), then **Start Tracker**.
3. Actions tab: **Start Inspect**.

Other objects: `target:=asymmetric_pipestar_with_cap | pipestar_without_pipes | object_robocup_pipestar | cube |
april_tag_cube` (tag size 80 mm), moved with `target_x`, `target_y`, `target_yaw`.

Note: dexterity inspects along the +Z axis of the object frame, which points **up** for the M3T / ICG models.
With side `origin` an upright pipestar gives a target 0.3 m above it, out of reach of the arm, so planning fails.
The `top/front/right/left` side offsets still have to be checked against the M3T / ICG model frame.

### Hazmat detection (optional)

```bash
ros2 launch hazmat_detection hazmat_detection.launch.py
```

### Quick checks

- **Dexterity:** **Start Inspect** in the Arm Control panel; the status ends at "INSPECT OPERATION - DONE".

## Dexterity in detail

```bash
ros2 launch dexterity_ur ur_sim.launch.py
```

This starts Gazebo (UR5e + Robotiq 2F-85 + wrist camera on a stand, AprilTag cube on a pedestal 0.9 m in front
of the arm), MoveIt 2 + RViz after 10 s, and the dexterity node and the **Dexterity Arm Control** panel after 20 s.
The arm then moves to "Home", where the wrist camera faces the cube (`home_on_start:=false` skips this).
In the panel: **Start Inspect** (and **Reset Arm** to go back to "Home"). The parameters are on the same page (see
[Arm Control panel](#arm-control-panel)); they are sent to the node with every button press and are remembered by rqt.

The panel can also be opened on its own: `rqt --standalone dexterity_gui/ArmControl` (or from the rqt menu
Plugins > Robot Tools). Or use the command line client in a second terminal:

```bash
ros2 run dexterity dexterity_client Reset        # arm to "Home": the wrist camera looks at the cube
ros2 run dexterity dexterity_client Inspection   # move to 0.3 m in front of the tag; Ctrl+C cancels
```

The client takes the same positional arguments as the ROS 1 client; only the command is required, the
rest comes from `dexterity_ur/config/ur5e.yaml`:

```bash
ros2 run dexterity dexterity_client Inspection 1 hybrid visp cube_rl2 dexterity_tool_frame half_trajectory 0 0 origin
```

Commands: `Inspection`, `Follow` (moving object), `Touch`, `Stop`, `Reset`. Results are appended to
`controller_log.txt` in the directory the launch file was started from.

## Arm Control panel

`rqt --standalone dexterity_gui/ArmControl`. The layout is `dexterity_gui/ui/ArmControl.ui`: edit it with
`designer dexterity_gui/ui/ArmControl.ui` and rebuild `dexterity_gui` (the file is compiled into the plugin). A new widget
needs a connection in `ArmControl.cpp`, a renamed one needs its `ui->name` references updated.

Every button sends one goal to the `dexterity_action` server with the command and **all** values of the forms below.
An empty text or a zero number means "keep the value from the robot YAML"; booleans always come from the panel.

### Actions

| Button | Command | What it does |
|---|---|---|
| Start Touch | `touch` | Moves to the inspection distance in front of the object, then a final approach to touch it |
| Start Follow | `inspect_non_stationary` | Keeps the inspection pose relative to a moving object; never enters the final sequence |
| Start Inspect | `inspect_stationary` | Moves to the inspection pose in front of a still object and finishes there |
| Start Grasp | `grasp` | Grasp operation |
| Stop | `stop` | Halts the current task |
| Reset Arm | `reset` | Returns the arm to its home state |

### Perception parameters

| Parameter | Meaning |
|---|---|
| Tracker Name | `visp`: ViSP model-based tracker inside the dexterity node. `icg` / `m3t`: external trackers, the node only listens to their poses on `/tracked_object_poses`. Also switches the tracker image topic of the Tracker view |
| Tracking Mode (ViSP) | Image features ViSP follows: `me` moving edges (contours, for textureless objects), `klt` keypoints (needs texture), `hybrid` both. `visp` only |
| Object Model File Name | CAD model (`.cao`) the tracker matches, listed from `dexterity/config/cad_models`. `cube`, `asymmetric_pipestar_*` are the old ICG / M3T body names and select the reference picture. Must match the real object |
| Object Side | Point of interest the gripper approaches: `origin` (model origin), `top`, `front`, `front_level`, `front_tilt`, `left`, `right`. Fixed offsets measured for one object, so they only fit that object |
| Min Accuracy Threshold (%) | Poses below this accuracy are invalid and trigger a new track, so the arm only moves on poses at or above it. Higher is more cautious but can stall the motion with weak tracking |

ViSP AprilTag bootstrapping (ViSP gets its first pose from an AprilTag; greyed out for `icg` / `m3t`):

| Parameter | Meaning |
|---|---|
| Tag Size (mm) | Edge length of the black square of the real tag. A wrong value gives a wrong distance |
| Tag Family | `36h11` (default), `25h9`, `16h5`, `standard41h12`. Must match the printed tag |
| Pose Estimation Method | `homography_virtual_vs` (default), `homography`, `dementhon_virtual_vs`, `lagrange_virtual_vs`, `best_virtual_vs`, `homography_orthogonal_iteration` |

An invalid tag size, family or method is logged as an error and the bootstrapper stays off.

### Controller parameters

| Parameter | Meaning |
|---|---|
| Open Loop Mode | Off (closed loop): plan, move part of the way, look again, replan; corrects tracking error and object motion. On: plan once, execute the whole trajectory, no re-check |
| Orientation Lock | Off: shortest rotation from the current gripper orientation. On: the gripper takes the exact orientation of the inspection pose |
| Inspection Distance (cm) | Stand-off distance between the point of interest and the gripper (sent in m) |
| Gripper Frame ID | TF frame used as the gripper; its +X axis must point out of the gripper. Default `dexterity_tool_frame` |
| Truncation Type | Closed loop only. `half_trajectory`: execute the first half of each plan, `quater_trajectory`: the first quarter, `step_by_step`: only 3 points. Smaller steps re-check more often but progress slower. The last point is set to rest because the joint trajectory controller rejects trajectories that end in motion |
| Inspection Pose Margin Error (mm) | Allowed difference between the distance to the object and the inspection distance for "pose reached" |
| Inspection Orientation Margin Error (deg) | The same for the orientation. Both margins must be met to start the final sequence (touch) or finish the inspection |
| Planning Timeout (ms) | Time MoveIt may spend on each plan. Longer finds harder plans but makes each step slower |

Open Loop, Orientation Lock, Distance and the two margins matter for the inspect and touch operations. The margins and
the distance are barely relevant for grasp.

Simulation or hardware is chosen by the launch file and robot YAML (`dexterity_ur`), not by the panel.

### Tracker panel (ICG / M3T manual detection)

| Button | Action |
|---|---|
| Start Bootstrapping | Freezes the camera image so the 6 detection points can be clicked |
| Stop Bootstrapping | Clears the points and unfreezes the image |
| Start Tracker | Publishes the 6 points and the model name on `GUI_MANUAL_DETECTION_POINTS` for the ICG / M3T tracker; refuses unless exactly 6 points are selected |
| Show Reference | Shows a picture of where to click for the selected model |

The camera and tracker image topics are hidden fields that are saved by rqt; they only affect the displayed image.

## Evaluation (simulation)

Measures dexterity against the true object pose from Gazebo, e.g. to compare ViSP, ICG and M3T. Start it next to
the simulation with the same `target`:

```bash
ros2 launch dexterity_ur ur_sim.launch.py target:=asymmetric_pipestar_without_cap
ros2 launch dexterity_evaluation evaluation.launch.py target:=asymmetric_pipestar_without_cap   # output_dir:=... optional
```

Every dexterity task (Start Inspect / Follow / Touch / Grasp until it finishes or is stopped) is recorded to
`~/dexterity_evaluation`:

| File | Contents |
|---|---|
| `<date>_<command>_<tracker>_samples.csv` | 10 Hz: tracking error of the latest tracker pose (mm, deg) and its age, true distance gripper -> point of interest, its error to the inspection distance (mm), approach angle error (deg), gripper position |
| `<date>_<command>_<tracker>_planning.csv` | every MoveIt plan: controller state, success, planning time, distance seen by dexterity |
| `summary.csv` | one line per task: result, duration, plans / failed plans, planning times, final true distance and approach errors, mean / max tracking errors |

The true pose is also published as TF frame `ground_truth_object` (visible in RViz). Dexterity decides "reached"
from the *tracked* pose, so the true final error can be larger than the 5 mm margin; the files show by how much.
`dexterity_evaluation/config/targets.yaml` holds, per target, where the tracked frame lies in the Gazebo model
(ViSP: tag face of the cube; M3T / ICG: model origin). A new target needs an entry there.
