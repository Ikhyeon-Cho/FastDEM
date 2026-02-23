"""Launch file for FastDEM elevation mapping node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    global_mapping = LaunchConfiguration('global_mapping').perform(context) == 'true'
    input_scan = LaunchConfiguration('input_scan').perform(context)

    # Package path
    pkg_share = FindPackageShare('fastdem_ros2')

    # Config file (single superset YAML — same format as ROS1)
    config_name = 'global_mapping.yaml' if global_mapping else 'local_mapping.yaml'
    rviz_name = 'fastdem_global.rviz' if global_mapping else 'fastdem_local.rviz'
    config_file = PathJoinSubstitution([pkg_share, 'config', config_name])
    rviz_config = PathJoinSubstitution([pkg_share, 'launch', 'rviz', rviz_name])

    # Node parameters
    node_params = {'config_file': config_file}
    if input_scan:
        node_params['input_scan'] = input_scan

    # FastDEM mapping node
    fastdem_node = Node(
        package='fastdem_ros2',
        executable='fastdem_node',
        name='fastdem',
        output='screen',
        parameters=[node_params],
    )

    # RViz2 (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [fastdem_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'global_mapping', default_value='false',
            description='Enable global (fixed-origin) mapping mode'),
        DeclareLaunchArgument(
            'input_scan', default_value='',
            description='Override input topic (empty = use config)'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Launch RViz2 for visualization'),
        OpaqueFunction(function=_launch_setup),
    ])
