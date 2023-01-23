from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    fcu_url_arg = LaunchConfiguration('fcu_url')
    gcs_url_arg = LaunchConfiguration('gcs_url')
    system_id_arg = LaunchConfiguration('system_id')
    component_id_arg = LaunchConfiguration('component_id')
    tgt_system_arg = LaunchConfiguration('target_system_id')
    tgt_component_arg = LaunchConfiguration('target_component_id')

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14551@:14555'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='udp://@localhost:14550'
    )

    system_id_arg = DeclareLaunchArgument(
        'system_id',
        default_value='255'
    )

    component_id_arg = DeclareLaunchArgument(
        'component_id',
        default_value='240'
    )

    tgt_system_arg = DeclareLaunchArgument(
        'target_system_id',
        default_value='1'
    )

    tgt_component_arg = DeclareLaunchArgument(
        'target_component_id',
        default_value='1'
    )

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://127.0.0.1:14551@14555'
        ],
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'system_id': LaunchConfiguration('system_id'),
            'component_id': LaunchConfiguration('component_id'),
            'target_system_id': LaunchConfiguration('target_system_id'),
            'target_component_id': LaunchConfiguration('target_component_id'),
        }]
    )
    return LaunchDescription([
        fcu_url_arg,
        gcs_url_arg,
        system_id_arg,
        component_id_arg,
        tgt_system_arg,
        tgt_component_arg,
        mavros_node
    ])
