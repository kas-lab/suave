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
    mc_reasoning_time_filename = LaunchConfiguration('mc_reasoning_time_filename')

    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge' +
                    '[none, metacontro, random, or bt]'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Desired mission type' +
                    '[time_constrained_mission or const_dist_mission (deprecated)]'
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

    mc_reasoning_time_filename_arg = DeclareLaunchArgument(
        'mc_reasoning_time_filename',
        default_value='metacontrol_reasoning_time',
        description='metacontrol reasoning time filename'
    )

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')
    suave_metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_metacontrol.launch.py'
    )

    pkg_suave_random_path = get_package_share_directory(
        'suave_random')
    suave_random_launch_path = os.path.join(
        pkg_suave_random_path,
        'launch',
        'suave_random.launch.py'
    )
    
    pkg_suave_none_path = get_package_share_directory(
        'suave_none')
    suave_none_launch_path = os.path.join(
        pkg_suave_none_path,
        'launch',
        'suave_none.launch.py'
    )
    
    pkg_suave_bt_path = get_package_share_directory(
        'suave_bt')
    suave_bt_launch_path = os.path.join(
        pkg_suave_bt_path,
        'launch',
        'suave_bt.launch.py'
    )

    suave_metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_metacontrol_launch_path),
        launch_arguments = {
            'reasoning_time_filename' : mc_reasoning_time_filename}.items(),
        condition=LaunchConfigurationEquals(
            'adaptation_manager', 'metacontrol'))

    suave_random_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_random_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'random'))
    
    suave_none_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_none_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'none'))
    
    suave_bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_bt_launch_path),
        condition=LaunchConfigurationEquals('adaptation_manager', 'bt'))

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
        result_filename_arg,
        battery_constraint_arg,
        battery_constraint_value_arg,
        mc_reasoning_time_filename_arg,
        suave_metacontrol_launch,
        suave_random_launch,
        suave_none_launch,
        suave_bt_launch,
    ])
