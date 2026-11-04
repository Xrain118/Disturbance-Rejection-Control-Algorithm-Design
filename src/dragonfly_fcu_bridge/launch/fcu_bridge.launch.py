from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("dragonfly_fcu_bridge")) / "config/fcu_bridge.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument("model_metadata", default_value=""),
        DeclareLaunchArgument("expected_model_version", default_value=""),
        Node(
            package="dragonfly_fcu_bridge", executable="fcu_bridge", name="fcu_bridge",
            parameters=[LaunchConfiguration("config"), {
                "model_metadata": LaunchConfiguration("model_metadata"),
                "expected_model_version": LaunchConfiguration("expected_model_version"),
            }],
            output="screen",
        ),
    ])
