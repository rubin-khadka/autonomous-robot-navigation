import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'autonomous_robot_navigation'
    
    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation time'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    # Path to your map in the package's maps folder
    map_path = PathJoinSubstitution([
        get_package_share_directory(package_name),
        'maps',
        'custom_warehouse.yaml'  
    ])

    # Nav2 bringup
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'bringup_launch.py'
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map': map_path,
            'params_file': PathJoinSubstitution([
                get_package_share_directory(package_name),
                'config',
                'navigation_parameters.yaml'
            ]),
            'autostart': 'true',
        }.items()
    )

    # RViz with Nav2 config
    rviz_config_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        nav2_launch,
        rviz_node,
    ])