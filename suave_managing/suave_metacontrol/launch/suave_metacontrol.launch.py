import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tomasys_file = LaunchConfiguration('tomasys_file')
    model_file = LaunchConfiguration('model_file')
    reasoning_time_filename = LaunchConfiguration('reasoning_time_filename')

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

    return LaunchDescription([
        tomasys_file_arg,
        model_file_arg,
        reasoning_time_filename_arg,
        mros_reasoner_node,
        mros_system_modes_bridge_node,
        task_bridge_node,
        suave_launch,
    ])
