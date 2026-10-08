# dexterity_ur

Runs the dexterity hand-eye controller on a simulated **UR5e** with a Robotiq 2F-85 and a wrist camera (ROS 2 Jazzy,
Gazebo Harmonic). Uses `ur_description`, `ur_simulation_gz`, `ur_moveit_config` and
`robotiq_description` (in `src/grippers`).

```bash
ros2 launch dexterity_ur ur_sim.launch.py
ros2 launch dexterity_ur ur_sim.launch.py target:=asymmetric_pipestar_without_cap   # object for M3T / ICG
ros2 launch dexterity_ur ur_sim.launch.py gazebo_gui:=false rviz:=false launch_gui:=false show_tracking_window:=false   # headless

ros2 run dexterity dexterity_client Reset          # arm to "Home", the camera looks at the target
ros2 run dexterity dexterity_client Inspection     # inspect the target (settings in config/ur5e.yaml)
```

Launch arguments: `target`, `target_x` (0.9), `target_y`, `target_yaw`, `world`, `rviz`, `gazebo_gui`, `start_dexterity`,
`launch_gui`, `show_tracking_window`, `home_on_start`.

## What is in here

| File | Purpose |
|---|---|
| `urdf/ur_camera.urdf.xacro` | UR5e on a 0.3 m stand + adapter + Robotiq 2F-85 + wrist camera (Gazebo sensor) + `camera_optical_link` + `dexterity_tool_frame` |
| `config/ur5e_controllers.yaml` | ros2_control controllers: arm (`scaled_joint_trajectory_controller`), gripper (`robotiq_gripper_controller`) |
| `config/ur5e.yaml` | dexterity parameters (planning group `ur_manipulator`, camera topics, frames, inspection distance) |
| `launch/ur_sim.launch.py` | Gazebo, controllers, camera bridge, target, MoveIt, dexterity, panel. Generates the SRDF (the `ur_moveit_config` one plus gripper collision rules and the `Home` state) |
| `worlds/` | `ur_world_featherstone.sdf` (default) and `ur_world.sdf` (DART) |
| `models/` | stand, pedestal, AprilTag cube and the pipestar objects |

## Things to know

- **Gripper and physics engine.** With DART (`world:=ur_world.sdf`) the simulated gripper freezes after it was sent to
  exactly 0.0, its lower joint limit, and no longer moves. Bullet Featherstone (the default world) has no such problem.
  With DART use 0.05 as open. Closed is 0.79 (0.8 is the joint limit). The gripper starts closed.
- **`Home`** is a joint state computed so that the gripper axis points along +X at 0.49 m above the arm base with the
  camera on top of the gripper (upright image). It lives in `HOME_JOINTS` in the launch file.
- **Reach.** The UR5e reaches 0.85 m. `target_x` 0.9 is fine because the arm only needs to get to 0.3 m from the target.
- **Do not add `position_proportional_gain` to the controllers file.** With the UR and this
  gripper it made the simulation vibrate violently.
- **Colons in xacro comments.** `ros2 launch` reads the xacro output as YAML, so a comment with a colon followed by a
  space breaks the launch ("Unable to parse the value of parameter robot_description as yaml").
