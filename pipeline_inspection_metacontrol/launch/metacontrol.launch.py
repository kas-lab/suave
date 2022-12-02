import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_mros2_reasoner_path = get_package_share_directory(
        'mros2_reasoner')

    mros2_launch_path = os.path.join(
        pkg_mros2_reasoner_path, 'launch_reasoner.launch.py')

    pkg_pipeline_inspection_metacontrol_path = get_package_share_directory(
        'pipeline_inspection_metacontrol')

    pipeline_inspection_ontology_path = os.path.join(
        pkg_pipeline_inspection_metacontrol_path,
        'config',
        'pipeline_inspection.owl')

    mros2_reasoner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mros2_launch_path),
        launch_arguments={
            'model_file': pipeline_inspection_ontology_path}.items())

    return LaunchDescription([
        mros2_reasoner_node
    ])
