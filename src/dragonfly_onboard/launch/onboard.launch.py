"""Hardware-only Dragonfly bringup; contains no Gazebo or simulation-time dependency."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arm_config = LaunchConfiguration("arm_config")
    bridge_config = LaunchConfiguration("bridge_config")
    model_metadata = LaunchConfiguration("model_metadata")
    expected_model_version = LaunchConfiguration("expected_model_version")
    start_fcu_bridge = LaunchConfiguration("start_fcu_bridge")

    serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dragonfly_serial"), "launch", "dual_arm_serial.launch.py"]
            )
        ),
        launch_arguments={"config": arm_config}.items(),
    )
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dragonfly_fcu_bridge"), "launch", "fcu_bridge.launch.py"]
            )
        ),
        condition=IfCondition(start_fcu_bridge),
        launch_arguments={
            "config": bridge_config,
            "model_metadata": model_metadata,
            "expected_model_version": expected_model_version,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("dragonfly_serial"), "config", "dual_arm_serial.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "bridge_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("dragonfly_fcu_bridge"), "config", "fcu_bridge.yaml"
            ]),
        ),
        DeclareLaunchArgument("model_metadata", default_value=""),
        DeclareLaunchArgument("expected_model_version", default_value=""),
        DeclareLaunchArgument(
            "start_fcu_bridge",
            default_value="false",
            description="Start physical-wrench to PX4 setpoint conversion after metadata is supplied.",
        ),
        serial_launch,
        bridge_launch,
    ])
