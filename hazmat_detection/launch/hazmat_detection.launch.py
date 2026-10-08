from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare("hazmat_detection"), "config", "hazmat_detection.yaml"])
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("image_topic", default_value="/wrist_mounted_camera/image"),
    ]
    node = Node(
        package="hazmat_detection",
        executable="hazmat_detection_node",
        name="hazmat_detection",
        output="screen",
        parameters=[
            config,
            {
                "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                "image_topic": LaunchConfiguration("image_topic"),
            },
        ],
    )
    return LaunchDescription(args + [node])
