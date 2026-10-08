"""
UR5e + Robotiq 2F-85 + wrist camera in Gazebo + MoveIt 2 + the dexterity node, with an AprilTag cube in front of the arm.

    ros2 launch dexterity_ur ur_sim.launch.py
    ros2 launch dexterity_ur ur_sim.launch.py target:=asymmetric_pipestar_without_cap   # object for M3T / ICG

The arm moves to "Home" automatically (home_on_start:=false to skip). Then use the "Dexterity Arm Control" panel
(Start Inspect), or in another terminal:

    ros2 run dexterity dexterity_client Reset          # move the arm to "Home" (camera looks at the cube)
    ros2 run dexterity dexterity_client Inspection     # inspect the cube (settings from config/ur5e.yaml)

Robot description, world, target models and parameters are all in this package.
"""

import itertools
import os
import tempfile

import xacro

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

UR_TYPE = "ur5e"

# Height of the top of the pedestals and the arm base (the stand is 0.299 m high, the arm base is at 0.3 m)
PEDESTAL_TOP = 0.7415
PEDESTAL_OBJECTS = {
    "asymmetric_pipestar_with_cap": (0.1, 0.0, 0.0, 0.0),
    "asymmetric_pipestar_without_cap": (0.0, 0.0, 0.0, 0.0),
    "pipestar_without_pipes": (0.075, 0.0, 0.0, 0.0),
    "object_robocup_pipestar": (0.001, 0.0, 0.0, 0.0),  # not static: the cap can be pulled off
    "cube": (0.05, 0.0, 0.0, 0.0),
    "april_tag_cube": (0.05, 0.0, -1.5707963, 0.0),
}

# "Home": the gripper (and the camera) points horizontally along +X at 0.49 m above the arm base, towards the target,
# with the camera on top of the gripper so that the image is upright. Computed with the forward kinematics of the UR5e,
# tool frame 0.156 m from the flange.
HOME_JOINTS = {
    "shoulder_pan_joint": -0.5774,
    "shoulder_lift_joint": -2.0208,
    "elbow_joint": 1.9073,
    "wrist_1_joint": 0.1131,
    "wrist_2_joint": 0.9929,
    "wrist_3_joint": 0.0,
}

# Gripper and adapter links, never in collision with each other or with the last arm links
GRIPPER_LINKS = [
    "ur_to_robotiq_link", "robotiq_85_base_link",
    "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
    "robotiq_85_left_inner_knuckle_link", "robotiq_85_right_inner_knuckle_link",
    "robotiq_85_left_finger_link", "robotiq_85_right_finger_link",
    "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link",
]
WRIST_LINKS = ["tool0", "flange", "wrist_3_link", "wrist_2_link", "wrist_1_link"]


def srdf_file():
    """SRDF of ur_moveit_config plus the gripper collision rules and the "Home" state, written to a temporary file."""
    stock = os.path.join(get_package_share_directory("ur_moveit_config"), "srdf", "ur.srdf.xacro")
    srdf = xacro.process_file(stock, mappings={"name": "ur"}).toprettyxml(indent="  ")

    extra = ['  <group_state name="Home" group="ur_manipulator">']
    extra += [f'    <joint name="{joint}" value="{value}"/>' for joint, value in HOME_JOINTS.items()]
    extra += ['  </group_state>']
    pairs = list(itertools.combinations(GRIPPER_LINKS, 2)) + list(itertools.product(GRIPPER_LINKS, WRIST_LINKS))
    extra += [f'  <disable_collisions link1="{a}" link2="{b}" reason="User"/>' for a, b in pairs]

    head, _, tail = srdf.rpartition("</robot>")
    target = os.path.join(tempfile.gettempdir(), "dexterity_ur_srdf.srdf")
    with open(target, "w") as f:
        f.write(head + "\n".join(extra) + "\n</robot>" + tail)
    return target


def spawn_target(context, models):
    """Spawns the target selected with target:=..., either apriltag_target or an object on a pedestal."""
    target = LaunchConfiguration("target").perform(context)
    x = LaunchConfiguration("target_x").perform(context)
    y = LaunchConfiguration("target_y").perform(context)
    yaw = float(LaunchConfiguration("target_yaw").perform(context))

    def create(model, name, z, roll=0.0, pitch=0.0, model_yaw=0.0):
        return Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-file", os.path.join(models, model, "model.sdf"), "-name", name,
                       "-x", x, "-y", y, "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", str(model_yaw + yaw)],
        )

    if target == "apriltag_target":
        return [create("apriltag_target", "apriltag_target", 0.0)]
    if target not in PEDESTAL_OBJECTS:
        raise RuntimeError(f"Unknown target '{target}'. Available: apriltag_target, {', '.join(PEDESTAL_OBJECTS)}")
    height, roll, pitch, model_yaw = PEDESTAL_OBJECTS[target]
    return [
        create("object_pedestal", "object_pedestal", 0.0),
        create(target, target, PEDESTAL_TOP + height, roll, pitch, model_yaw),
    ]


def generate_launch_description():
    share = get_package_share_directory("dexterity_ur")
    models = os.path.join(share, "models")
    urdf = os.path.join(share, "urdf", "ur_camera.urdf.xacro")
    controllers = os.path.join(share, "config", "ur5e_controllers.yaml")
    dexterity_config = os.path.join(share, "config", "ur5e.yaml")

    rviz = LaunchConfiguration("rviz")
    start_dexterity = LaunchConfiguration("start_dexterity")
    launch_gui = LaunchConfiguration("launch_gui")
    home_on_start = LaunchConfiguration("home_on_start")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world = LaunchConfiguration("world")
    show_tracking_window = LaunchConfiguration("show_tracking_window")

    arguments = [
        DeclareLaunchArgument("target", default_value="asymmetric_pipestar_with_cap",
                              description="Object in front of the arm, apriltag_target or " + ", ".join(PEDESTAL_OBJECTS)
                              + " (on a pedestal)"),
        DeclareLaunchArgument("target_x", default_value="0.9", description="X of the target (m, robot base at the origin)"),
        DeclareLaunchArgument("target_y", default_value="0.0", description="Y of the target (m)"),
        DeclareLaunchArgument("target_yaw", default_value="0.0", description="Rotation of the target about Z (rad)"),
        DeclareLaunchArgument("wall", default_value="true", description="Add a wall behind the target"),
        DeclareLaunchArgument("wall_x", default_value="1.5", description="X of the wall (m, robot base at the origin)"),
        DeclareLaunchArgument("rviz", default_value="true", description="Start RViz with the MoveIt plugin"),
        DeclareLaunchArgument("world", default_value="ur_world_featherstone.sdf",
                              description="World in this package's worlds folder, ur_world_featherstone.sdf (Bullet "
                              "Featherstone physics) or ur_world.sdf (Gazebo default, DART). With DART the gripper "
                              "freezes after it was sent to exactly 0.0, so use 0.05 as open"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", description="Open the Gazebo window"),
        DeclareLaunchArgument("start_dexterity", default_value="true", description="Start the dexterity node"),
        DeclareLaunchArgument("launch_gui", default_value="true", description="Open the Dexterity Arm Control panel (rqt)"),
        DeclareLaunchArgument("show_tracking_window", default_value="true", description="ViSP window with the tracked object"),
        DeclareLaunchArgument("home_on_start", default_value="true",
                              description="Move the arm to 'Home' once dexterity is up, so the wrist camera faces the target"),
    ]

    # Gazebo finds the target models and the gripper meshes (package://robotiq_description/...) through this path
    gz_resource_path = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", models),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(get_package_prefix("robotiq_description"), "share")),
    ]

    # UR5e + gripper + camera in Gazebo with ros2_control (ur_simulation_gz with our description, controllers and world)
    robot_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ur_simulation_gz"), "launch", "ur_sim_control.launch.py")),
        launch_arguments={
            "ur_type": UR_TYPE,
            "description_file": urdf,
            "controllers_file": controllers,
            "world_file": PathJoinSubstitution([share, "worlds", world]),
            "launch_rviz": "false",
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    # The gripper controller is not started by ur_sim_control.launch.py
    gripper_controller = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["robotiq_gripper_controller", "-c", "/controller_manager", "--controller-manager-timeout", "120"],
    )

    camera_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=["/wrist_mounted_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                   "/wrist_mounted_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        parameters=[{"use_sim_time": True}],
    )

    # The arm base is at z = 0.3 m (see the URDF), so give it a stand
    spawn_stand = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-file", os.path.join(models, "robot_stand", "model.sdf"), "-name", "robot_stand",
                   "-x", "0.0", "-y", "0.0", "-z", "0.0"],
    )

    spawn_wall = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-file", os.path.join(models, "wall", "model.sdf"), "-name", "wall",
                   "-x", LaunchConfiguration("wall_x"), "-y", "0.0", "-z", "0.0"],
        condition=IfCondition(LaunchConfiguration("wall")),
    )

    # MoveIt configuration (move_group and the MoveGroupInterface inside dexterity use the same one)
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(file_path=urdf, mappings={"name": "ur", "ur_type": UR_TYPE})
        .robot_description_semantic(file_path=srdf_file())
        .to_moveit_configs()
    )
    # The stock 5 ms KDL timeout is too short for pose targets of a 6 DoF arm (a 7 DoF arm would not need this)
    moveit_config.robot_description_kinematics["robot_description_kinematics"]["ur_manipulator"]["kinematics_solver_timeout"] = 0.05

    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True, "publish_robot_description_semantic": True}],
    )
    moveit_rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2_moveit", output="log",
        arguments=["-d", os.path.join(get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(rviz),
    )

    dexterity = Node(
        package="dexterity", executable="dexterity_node", name="dexterity", output="screen",
        parameters=[moveit_config.to_dict(), dexterity_config, {"use_sim_time": True},
                    {"perception.show_tracking_window": ParameterValue(PythonExpression(["'", show_tracking_window, "' == 'true'"]), value_type=bool)}],
        condition=IfCondition(start_dexterity),
    )

    # Operator panel (ROS 2 port of the ArmControl widget)
    gui = Node(
        package="rqt_gui", executable="rqt_gui", name="dexterity_gui",
        arguments=["--force-discover", "--standalone", "dexterity_gui/ArmControl"],
        condition=IfCondition(launch_gui),
    )

    # The arm starts upright. Moving to "Home" turns the camera towards the target.
    # The service call waits until the dexterity node offers arm_home.
    move_home = ExecuteProcess(
        cmd=["ros2", "service", "call", "/arm_home", "dexterity_msgs/srv/ArmHome"],
        name="move_home", output="screen",
        condition=IfCondition(PythonExpression(["'", home_on_start, "' == 'true' and '", start_dexterity, "' == 'true'"])),
    )

    return LaunchDescription(arguments + gz_resource_path + [
        robot_sim,
        gripper_controller,
        camera_bridge,
        spawn_stand,
        spawn_wall,
        OpaqueFunction(function=spawn_target, args=[models]),
        TimerAction(period=10.0, actions=[move_group, moveit_rviz]),  # after the controllers are up
        TimerAction(period=20.0, actions=[dexterity]),                # after move_group is up
        TimerAction(period=20.0, actions=[gui, move_home]),
    ])
