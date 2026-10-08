from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("pose_estimation_icg"), "config", "icg.yaml"])
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_depth", default_value="false"),
        DeclareLaunchArgument("default_body", default_value="asymmetric_pipestar_without_cap"),
        DeclareLaunchArgument("display_images", default_value="false"),
    ]
    node = Node(
        package="pose_estimation_icg",
        executable="icg_tracker_node",
        name="pose_estimation_icg",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "use_depth": ParameterValue(LaunchConfiguration("use_depth"), value_type=bool),
                "default_body": LaunchConfiguration("default_body"),
                "display_images": ParameterValue(LaunchConfiguration("display_images"), value_type=bool),
            },
        ],
    )
    return LaunchDescription(args + [node])
