"""
Ground truth evaluation of dexterity in the Gazebo simulation. Start it next to the simulation:

    ros2 launch dexterity_ur ur_sim.launch.py target:=asymmetric_pipestar_without_cap
    ros2 launch dexterity_evaluation evaluation.launch.py target:=asymmetric_pipestar_without_cap

Every dexterity task is recorded to output_dir (default ~/dexterity_evaluation): <run>_samples.csv,
<run>_planning.csv and a line in summary.csv.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    target = LaunchConfiguration("target").perform(context)
    with open(os.path.join(get_package_share_directory("dexterity_evaluation"), "config", "targets.yaml")) as f:
        offsets = yaml.safe_load(f)
    if target and target not in offsets:
        print(f"[evaluation.launch.py] No object offset for target '{target}' in config/targets.yaml, using the model origin")
    return [Node(
        package="dexterity_evaluation",
        executable="evaluation_node",
        name="dexterity_evaluation",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "object_model": target,
            "object_offset": [float(v) for v in offsets.get(target, [0.0] * 6)],
            "output_dir": LaunchConfiguration("output_dir").perform(context),
            "world_name": LaunchConfiguration("world_name").perform(context),
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target", default_value="",
                              description="Gazebo model name of the object (target of ur_sim.launch.py); empty: detect"),
        DeclareLaunchArgument("output_dir", default_value="~/dexterity_evaluation"),
        DeclareLaunchArgument("world_name", default_value="empty"),
        OpaqueFunction(function=launch_setup),
    ])
