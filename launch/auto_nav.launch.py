import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter 
import xacro


def generate_launch_description():

    robotName = 'autonomous_robot'
    packageName = 'autonomous_robot_navigation'
    modelFilePath = 'urdf/autonomous_robot.urdf'

    pathModelFile = os.path.join(get_package_share_directory(packageName), modelFilePath)
    robotDescription = xacro.process_file(pathModelFile).toxml()

    set_use_sim_time = SetParameter(name='use_sim_time', value=True)

    world_path = os.path.join(
        get_package_share_directory(packageName),
        'worlds',
        'custom_warehouse.sdf'
    )

    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    )

    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch, 
        launch_arguments={
            'gz_args': f'{world_path} -r -v4',
            'on_exit_shutdown': 'true'
        }.items()  
    )

    spawnModelNodeGazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotName,
            '-topic', 'robot_description',
            "-x", "9.0", "-y", "7.0", "-z", "0.5", "-Y", "3.1415"
        ],
        output='screen',
    )

    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }]
    )

    bridge_params = os.path.join(
        get_package_share_directory(packageName),
        'config',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',  
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
            '-p', 'use_sim_time:=true'
        ],
        output='screen', 
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(set_use_sim_time)
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject