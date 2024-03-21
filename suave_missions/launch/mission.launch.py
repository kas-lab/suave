import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    adaptation_manager = LaunchConfiguration('adaptation_manager')
    mission_type = LaunchConfiguration('mission_type')
    result_filename = LaunchConfiguration('result_filename')
    battery_constraint = LaunchConfiguration('battery_constraint')
    battery_constraint_value = LaunchConfiguration('battery_constraint_value')

    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge' +
                    '[none or metacontrol or random]'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Desired mission type' +
                    '[time_constrained_mission or const_dist_mission]'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='',
        description='Name of the results file'
    )

    battery_constraint_arg = DeclareLaunchArgument(
        'battery_constraint',
        default_value='False',
        description='Desired battery functionality' +
                    '[True or False]'
    )
    battery_constraint_value_arg = DeclareLaunchArgument(
        'battery_constraint_value',
        default_value='0.2',
        description='battery constraint value'
    )

    pkg_suave_path = get_package_share_directory(
        'suave_missions')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_metrics_node = Node(
        package='suave_missions',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': adaptation_manager,
            'mission_name': mission_type,
        }],
        condition=LaunchConfigurationEquals('result_filename', '')
    )

    mission_metrics_node_override = Node(
        package='suave_missions',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': adaptation_manager,
            'mission_name': mission_type,
            'result_filename': result_filename,
        }],
        condition=LaunchConfigurationNotEquals('result_filename', '')
    )

    mission_node = Node(
        package='suave_missions',
        executable=mission_type,
        name='mission_node',
        parameters=[
            mission_config,
            {
                'battery_constraint': battery_constraint,
                'battery_constraint_value': battery_constraint_value,
            }],
    )

    pkg_suave_path = get_package_share_directory('suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py'
    )

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')
    suave_metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_metacontrol.launch.py'
    )

    suave_random_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_random.launch.py'
    )

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'none'))

    suave_metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_metacontrol_launch_path),
        condition=LaunchConfigurationEquals(
            'adaptation_manager', 'metacontrol'))

    suave_random_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_random_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'random'))

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
        result_filename_arg,
        battery_constraint_arg,
        battery_constraint_value_arg,
        mission_metrics_node,
        mission_metrics_node_override,
        mission_node,
        suave_launch,
        suave_metacontrol_launch,
        suave_random_launch,
    ])
