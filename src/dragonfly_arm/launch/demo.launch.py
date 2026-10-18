"""Launch the vector platform and demonstrate dual-arm and tilt motions once."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("dragonfly_arm"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "paused": paused,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    demo_node = Node(
        package="dragonfly_arm",
        executable="demo_trajectory",
        name="demo_trajectory",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo graphical client.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo with physics paused. Keep false for the demo.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            gazebo_launch,
            demo_node,
        ]
    )
