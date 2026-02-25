from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vrx_navigation',
            executable='kinematics_filter',
            name='kinematics_filter_node'
        ),
        Node(
            package='vrx_navigation',
            executable='los_guidance',
            name='LOS_node'
        ),
        Node(
            package='vrx_navigation',
            executable='pid_controller',
            name='pid_stationkeep'
        ),
        Node(
            package='vrx_navigation',
            executable='map_viz',
            name='vrx_grid_plotter'
        )
    ])