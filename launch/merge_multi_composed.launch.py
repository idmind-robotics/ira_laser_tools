import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    declared_arguments = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                get_package_share_directory("ira_laser_tools"),
                "config",
                "laserscan_merge.yaml",
            ),
            description="Path to param config in yaml format",
        ),
    ]

    container = ComposableNodeContainer(
        name="ira_laser_tools_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="ira_laser_tools",
                plugin="ira_laser_tools::LaserscanMerger",
                name="laserscan_multi_merger",
                parameters=[params_file],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="both",
    )

    return LaunchDescription(declared_arguments + [container])
