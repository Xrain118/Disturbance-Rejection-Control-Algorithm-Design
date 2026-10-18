from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    config = str(Path(get_package_share_directory('dragonfly_serial')) / 'config/dual_arm_serial.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        Node(package='dragonfly_serial', executable='dual_arm_serial',
             name='dual_arm_serial', parameters=[LaunchConfiguration('config')], output='screen')])
