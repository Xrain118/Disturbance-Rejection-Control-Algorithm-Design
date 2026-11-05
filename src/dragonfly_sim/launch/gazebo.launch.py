"""Launch the fixed-base vector platform and dual arms in Gazebo Harmonic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    use_sim_time = LaunchConfiguration("use_sim_time")

    package_share = FindPackageShare("dragonfly_arm")
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "dragonfly_arm.urdf.xacro"]
    )
    world_file = PathJoinSubstitution(
        [FindPackageShare("dragonfly_sim"), "worlds", "model_preview.world"]
    )
    controller_file = PathJoinSubstitution(
        [package_share, "config", "controllers.yaml"]
    )
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " \"",
                xacro_file,
                "\" use_sim_time:=",
                use_sim_time,
            ]
        ),
        value_type=str,
    )

    # -s selects server-only mode; omitting -r starts the world paused.
    server_only_arg = PythonExpression(
        ["' -s' if '", gui, "'.lower() == 'false' else ''"]
    )
    run_arg = PythonExpression(
        ["'' if '", paused, "'.lower() == 'true' else ' -r'"]
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [server_only_arg, run_arg, " -v 2 \"", world_file, "\""],
            "gz_version": "8",
            "on_exit_shutdown": "true",
        }.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": use_sim_time}
        ],
    )

    kinematics_node = Node(
        package="dragonfly_arm",
        executable="kinematics_node",
        name="roarm_m3_kinematics",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_dragonfly_arm",
        output="screen",
        arguments=[
            "-world",
            "default",
            "-name",
            "dragonfly_arm",
            "-allow_renaming",
            "false",
            "-topic",
            "robot_description",
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    def trajectory_spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "60",
                "--param-file",
                controller_file,
            ],
            output="screen",
        )

    left_arm_controller_spawner = trajectory_spawner("left_arm_controller")
    right_arm_controller_spawner = trajectory_spawner("right_arm_controller")
    tilt_controller_spawner = trajectory_spawner("tilt_controller")

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    start_motion_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                left_arm_controller_spawner,
                right_arm_controller_spawner,
                tilt_controller_spawner,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo Harmonic graphical client.",
            ),
            DeclareLaunchArgument(
                "paused",
                default_value="false",
                description="Start Gazebo with physics paused.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use the Gazebo simulation clock.",
            ),
            gazebo,
            clock_bridge,
            robot_state_publisher,
            kinematics_node,
            spawn_robot,
            start_joint_state_broadcaster,
            start_motion_controllers,
        ]
    )
