import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    reasoner_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'launch_reasoner.launch.py')

    suave_ontology_path = os.path.join(
        pkg_suave_metacontrol_path,
        'config',
        'suave.owl')

    mros2_reasoner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(reasoner_launch_path),
        launch_arguments={
            'model_file': suave_ontology_path}.items())

    return LaunchDescription([
        mros2_reasoner_node,
    ])
