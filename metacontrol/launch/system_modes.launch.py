from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, 
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():

    shm_model_path = (get_package_share_directory('metacontrol') +
        '/config/pipeline_modes.yaml')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_sime_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'),

    declare_desired_configuration_cmd = DeclareLaunchArgument(
        'desired_configuration',
        default_value='f_normal_mode',
        description='Desired inital configuration (system mode)')

    # Start as a normal node is currently not possible.
    # Path to SHM file should be passed as a ROS parameter.
    
    mode_manager_node = Node(
        package='metacontrol',
        executable='mode_manager',
        parameters=[{'modelfile': shm_model_path}],
        output='screen')

    dummy_lifecycle_node = LifecycleNode(
        package='metacontrol',
        executable='dummy_lifecycle',
        name='dummy_lifecycle_node',
        namespace='',
        output='screen')

    # autostart = LaunchConfiguration('autostart')
    # use_sim_time = LaunchConfiguration('use_sim_time')

    # param_substitutions = {
            # 'use_sim_time': use_sim_time,
            # 'autostart': autostart}

    # lifecycle_nodes = ['search',
                       # 'thruster']

    # lifecycle_manager_node = Node(
        # package='nav2_lifecycle_manager',
        # executable='lifecycle_manager',
        # name='lifecycle_manager_navigation',
        # output='screen',
        # parameters=[{'use_sim_time': use_sim_time},
                    # {'autostart': autostart},
                    # {'node_names': lifecycle_nodes}]),


    return LaunchDescription([
        mode_manager_node,
        dummy_lifecycle_node
    ])
