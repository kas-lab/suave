import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    result_path = LaunchConfiguration('result_path')
    result_filename = LaunchConfiguration('result_filename')
    time_limit = LaunchConfiguration('time_limit')
    adapt_manager = LaunchConfiguration('adapt_manager')
    f_generate_search_path_mode = LaunchConfiguration('f_generate_search_path_mode')
    f_inspect_pipeline_mode = LaunchConfiguration('f_inspect_pipeline_mode')
    mission_type = LaunchConfiguration('mission_type')
    adapt_period = LaunchConfiguration("adaptation_period")

    adapt_period_arg = DeclareLaunchArgument(
        'adaptation_period',
        default_value='15',
        description='How often the random adaptation happens every x (seconds)'
    )

    adapt_manager_arg = DeclareLaunchArgument(
        'adapt_manager',
        default_value='none',
        description='Which adaptation manager is in charge, none/metacontrol/random'
    )

    result_path_arg = DeclareLaunchArgument(
        'result_path',
        default_value='~/suave/results',
        description='Path to save mission measured metrics'
    )

    result_filename_arg = DeclareLaunchArgument(
        'result_filename',
        default_value='time_constrained_mission_results',
        description='Filename for the mission measured metrics'
    )

    time_limit_arg = DeclareLaunchArgument(
        'time_limit',
        default_value='300',
        description='IF a time constrained mission then time limit for the mission (seconds)'
    )

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )
    
    f_generate_search_path_mode_arg = DeclareLaunchArgument(
        'f_generate_search_path_mode',
        default_value='fd_spiral_low',
        description='Desired mode for f_generate_search_path_mode'
    )

    f_inspect_pipeline_mode_arg = DeclareLaunchArgument(
        'f_inspect_pipeline_mode',
        default_value='fd_inspect_pipeline',
        description='Desired mode for f_inspect_pipeline_mode'
    )

    pkg_suave_path = get_package_share_directory(
        'suave_missions')


    mission_launch_path = [PathJoinSubstitution([pkg_suave_path, 'launch', adapt_manager]), TextSubstitution(text='_mission.launch.py')]
    LogInfo(msg=mission_launch_path)


    # mission_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(mission_launch_path),
    #     launch_arguments={
    #         'mission_type': mission_type,
    #         'result_path': result_path,
    #         'result_filename': result_filename,
    #         'time_limit': time_limit,
    #         'f_generate_search_path_mode': f_generate_search_path_mode,
    #         'f_inspect_pipeline_mode': f_inspect_pipeline_mode,
    #         'adapt_period': adapt_period,
    #         'adapt_manager': adapt_manager,

    #     }.items())

    mission_node = Node(
        package='suave_missions',
        executable=mission_type,
        name=mission_type,
        parameters=[{
            'mission_type': mission_type,
            'result_path': result_path,
            'result_filename': result_filename,
            'time_limit': time_limit,
            'f_generate_search_path_mode': f_generate_search_path_mode,
            'f_inspect_pipeline_mode': f_inspect_pipeline_mode,
            'adapt_period': adapt_period,
            'adapt_manager': adapt_manager,
        }]
    )

    pkg_suave_path = get_package_share_directory(
        'suave')
    suave_launch_path = os.path.join(
        pkg_suave_path,
        'launch',
        'suave.launch.py')


    water_visibility_period = LaunchConfiguration('water_visibility_period')
    water_visibility_min = LaunchConfiguration('water_visibility_min')
    water_visibility_max = LaunchConfiguration('water_visibility_max')
    water_visibility_sec_shift = LaunchConfiguration(
        'water_visibility_sec_shift')

    water_visibility_period_arg = DeclareLaunchArgument(
        'water_visibility_period',
        default_value='100',
        description='Water visibility period in seconds'
    )

    water_visibility_min_arg = DeclareLaunchArgument(
        'water_visibility_min',
        default_value='1.25',
        description='Minimum value for water visibility'
    )

    water_visibility_max_arg = DeclareLaunchArgument(
        'water_visibility_max',
        default_value='3.75',
        description='Maximum value for water visibility'
    )

    water_visibility_sec_shift_arg = DeclareLaunchArgument(
        'water_visibility_sec_shift',
        default_value='0.0',
        description='Water visibility seconds shift to left'
    )

    thruster_events = LaunchConfiguration('thruster_events')

    thruster_events_arg = DeclareLaunchArgument(
        'thruster_events',
        default_value=str(['(1, failure,30)', '(2, failure,30)']),
        description='(thrusterN, failure/recovery, delta time in seconds ),' +
        ' e.g. (1, failure, 50)'
    )
    suave_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(suave_launch_path),
        launch_arguments={
            'water_visibility_period': water_visibility_period,
            'water_visibility_min': water_visibility_min,
            'water_visibility_max': water_visibility_max,
            'water_visibility_sec_shift': water_visibility_sec_shift,
            'thruster_events': thruster_events,
        }.items(),
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
        water_visibility_sec_shift_arg,
        water_visibility_max_arg,
        water_visibility_min_arg,
        water_visibility_period_arg,
        thruster_events_arg,
        adapt_manager_arg,
        adapt_period_arg,
        result_path_arg,
        result_filename_arg,
        time_limit_arg,
        mission_type_arg,
        f_generate_search_path_mode_arg,
        f_inspect_pipeline_mode_arg,
        mission_node,
        metacontrol_launch,
        suave_launch,
    ])