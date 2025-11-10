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

    ekf_params = os.path.join(
        get_package_share_directory(packageName),
        'config',
        'ekf.yaml'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params,
            {'use_sim_time': True}
        ]
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

    # # RGB image bridge with compression
    # gz_image_bridge_rgb = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["camera/image"],  
    #     output="screen",
    #     parameters=[{'use_sim_time': True}],
    # )

    # # Depth image bridge with compression  
    # gz_image_bridge_depth = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge", 
    #     arguments=["camera/depth_image"],  
    #     output="screen",
    #     parameters=[{'use_sim_time': True}],
    # )

    # # Camera info relays for compressed topics
    # relay_rgb_info = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='relay_rgb_camera_info',
    #     arguments=['camera/camera_info', 'camera/image/camera_info'],
    #     parameters=[{'use_sim_time': True}]
    # )

    # relay_depth_info = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='relay_depth_camera_info',
    #     arguments=['camera/camera_info', 'camera/depth_image/camera_info'],
    #     parameters=[{'use_sim_time': True}]
    # )


    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(set_use_sim_time)
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNodeGazebo)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)
    launchDescriptionObject.add_action(ekf_node)
    # launchDescriptionObject.add_action(gz_image_bridge_rgb)
    # launchDescriptionObject.add_action(gz_image_bridge_depth)
    # launchDescriptionObject.add_action(relay_rgb_info)
    # launchDescriptionObject.add_action(relay_depth_info)

    return launchDescriptionObject