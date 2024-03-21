from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    adaptation_manager_arg = DeclareLaunchArgument(
        'adaptation_manager',
        default_value='none',
        description='Adaptation manager in charge, none/metacontrol/random')

    mission_type_arg = DeclareLaunchArgument(
        'mission_type',
        default_value='time_constrained_mission',
        description='Which type of mission to have, time or distance'
    )

    return LaunchDescription([
        adaptation_manager_arg,
        mission_type_arg,
    ])
