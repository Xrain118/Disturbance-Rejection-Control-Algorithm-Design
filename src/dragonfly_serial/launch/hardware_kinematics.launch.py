from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    remaps = [('/joint_states', '/hardware/joint_states')]
    for side in ('left', 'right'):
        remaps += [(f'/{side}_arm/tool_pose', f'/hardware/{side}_arm/tool_pose'),
                   (f'/{side}_arm/solve_ik', f'/hardware/{side}_arm/solve_ik')]
    return LaunchDescription([Node(package='dragonfly_arm', executable='kinematics_node',
        name='hardware_kinematics', parameters=[{'use_sim_time': False}], remappings=remaps)])
