import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    adapt_manager = LaunchConfiguration('adapt_manager')
    mission_type = LaunchConfiguration('mission_type')

    adapt_manager_arg = DeclareLaunchArgument(
        'adapt_manager',
        default_value='none',
        description='Which adaptation manager is in charge, none/metacontrol/random'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )
    
    pkg_suave_path = get_package_share_directory(
        'suave_missions')

    mission_config = os.path.join(
        get_package_share_directory('suave_missions'),
        'config',
        'mission_config.yaml')


    mission_node = Node(
        package='suave_missions',
        executable=mission_type,
        name='parent_mission_node',
        parameters=[mission_config, {
            'mission_type': mission_type,
            'adapt_manager': adapt_manager,
        }]
    )

    pkg_suave_path = get_package_share_directory(
        'suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')

    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        # launch_arguments={
        #     'thruster_events': thruster_events,
        # }.items(),
        condition=LaunchConfigurationNotEquals('adapt_manager','metacontrol'))

    pkg_suave_metacontrol_path = get_package_share_directory(
        'suave_metacontrol')

    metacontrol_launch_path = os.path.join(
        pkg_suave_metacontrol_path,
        'launch',
        'suave_metacontrol.launch.py')

    metacontrol_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(metacontrol_launch_path),
        condition=LaunchConfigurationEquals('adapt_manager','metacontrol'),)

    # random_launch_path = os.path.join( #this is future-proofing, in a newer commit this would already work.
    #     pkg_suave_metacontrol_path,
    #     'launch',
    #     'suave_random.launch.py')

    # random_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(random_launch_path),
        # metacontrol_launch = IncludeLaunchDescription(
        # PythonLaunchDescriptionSource(metacontrol_launch_path),
        # launch_arguments={
        #     'adapt_period': something...dothislater

        # }.items(),
    #     condition=LaunchConfigurationEquals('adapt_manager','random'),)


    return LaunchDescription([
        adapt_manager_arg,
        mission_type_arg,
        mission_node,
        metacontrol_launch,
        suave_launch,
    ])