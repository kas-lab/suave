import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    silent = LaunchConfiguration('silent')
    silent_arg = DeclareLaunchArgument(
        'silent',
        default_value='false',
        description='Suppress all output (launch logs + node logs)'
    )
    def configure_logging(context, *args, **kwargs):
        if silent.perform(context) == 'true':
            import logging
            logging.getLogger().setLevel(logging.CRITICAL)
        return []

    tomasys_file = LaunchConfiguration('tomasys_file')
    model_file = LaunchConfiguration('model_file')
    reasoning_time_filename = LaunchConfiguration('reasoning_time_filename')
    battery_constraint = LaunchConfiguration('battery_constraint')
    battery_constraint_value = LaunchConfiguration('battery_constraint_value')
    result_filename = LaunchConfiguration('result_filename')

    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    tomasys_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl')]

    tomasys_file_arg = DeclareLaunchArgument(
        'tomasys_file',
        default_value=str(tomasys_files_array),
        description='Path for the tomasys ontologies'
    )

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    suave_ontology_path = os.path.join(
        pkg_suave_metacontrol_path,
        'config',
        'suave.owl')

    model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value=suave_ontology_path,
        description='Path for the application-specific ontology file')

    reasoning_time_filename_arg = DeclareLaunchArgument(
        'reasoning_time_filename',
        default_value='metacontrol_reasoning_time',
        description='File name for saving metacontrol reasoning time')
    
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

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='metacontrol_results',
        description='Name of the results file'
    )

    metacontrol_config = os.path.join(
        pkg_suave_metacontrol_path,
        'config',
        'metacontrol_config.yaml')

    mros_reasoner_node = Node(
        package='mros2_reasoner',
        executable='mros2_reasoner_node',
        name='suave_reasoner',
        output='screen',
        parameters=[metacontrol_config, {
            'tomasys_file': tomasys_file,
            'model_file': model_file,
            'reasoning_time_filename': reasoning_time_filename,
        }],
    )

    mros_system_modes_bridge_node = Node(
        package='mros2_reasoner',
        executable='mros2_system_modes_bridge',
    )

    task_bridge_node = Node(
        package='suave_metacontrol',
        executable='task_bridge_metacontrol',
    )

    pkg_suave_path = get_package_share_directory(
        'suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'task_bridge': 'False'}.items()
    )

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml'
    )

    mission_node = Node(
        package='suave_missions',
        executable='time_constrained_mission',
        name='mission_node',
        parameters=[
            mission_config,
            {
                'battery_constraint': battery_constraint,
                'battery_constraint_value': battery_constraint_value,
            }],
    )

    mission_metrics_node = Node(
        package='suave_metrics',
        executable='mission_metrics',
        name='mission_metrics',
        parameters=[mission_config, {
            'adaptation_manager': 'metacontrol',
            'mission_name': 'suave',
            'result_filename': result_filename,
        }],
    )

    return LaunchDescription([
        tomasys_file_arg,
        model_file_arg,
        reasoning_time_filename_arg,
        battery_constraint_arg,
        battery_constraint_value_arg,
        result_filename_arg,
        silent_arg,
        OpaqueFunction(function=configure_logging),
        mros_reasoner_node,
        mros_system_modes_bridge_node,
        task_bridge_node,
        suave_launch,
        mission_node,
        mission_metrics_node
    ])
