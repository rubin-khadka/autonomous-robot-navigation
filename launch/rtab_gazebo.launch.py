#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Configuration for YOUR robot
    localization = LaunchConfiguration('localization')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ⚡ USING YOUR ACTUAL TOPICS:
    rgb_topic = LaunchConfiguration('rgb_topic', default='/camera/image')
    depth_topic = LaunchConfiguration('depth_topic', default='/camera/depth_image')
    camera_info_topic = LaunchConfiguration('camera_info_topic', default='/camera/camera_info')
    scan_topic = LaunchConfiguration('scan_topic', default='/scan')
    odom_topic = LaunchConfiguration('odom_topic', default='/odometry/filtered')

    parameters={
        'frame_id': 'base_footprint',  # Your robot's base frame
        'odom_frame_id': 'odom',
        'use_sim_time': use_sim_time,
        'subscribe_rgbd': True,
        'subscribe_scan': True,  # You have LIDAR - using it!
        'subscribe_odom_info': False,
        'approx_sync': True,
        'sync_queue_size': 50,
        
        # RTAB-Map parameters optimized for YOUR setup
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true', 
        'RGBD/ProximityByTime': 'false',
        'Reg/Strategy': '1',  # Using ICP (good with LIDAR)
        'Vis/MinInliers': '12',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'RGBD/OptimizeMaxError': '3',
        'Reg/Force3DoF': 'true',  # Ground robot
        'Grid/FromDepth': 'true',
        'Grid/RangeMax': '3.0',
        'Grid/RangeMin': '0.1',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '1.0',
        'Mem/STMSize': '30',
        'RGBD/LocalRadius': '3',
        'Icp/CorrespondenceRatio': '0.2',
        'Icp/PointToPlane': 'true',
        'Icp/MaxCorrespondenceDistance': '0.1',
        'Icp/VoxelSize': '0.05',
        
        # Use both camera and LIDAR for best results
        'Grid/Sensor': '2',  # 0=laser, 1=RGB-D, 2=both
    }
    
    # ⚡ EXACT TOPIC REMAPPINGS FOR YOUR ROBOT:
    remappings=[
        ('rgb/image', rgb_topic),
        ('depth/image', depth_topic),
        ('rgb/camera_info', camera_info_topic),
        ('scan', scan_topic),
        ('odom', odom_topic)]
    
    # Optional: Use a custom RViz config
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('rtabmap_viz', default_value='true', description='Launch RTAB-Map UI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ.'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch in localization mode.'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock.'),
        
        # Your robot's specific topic arguments
        DeclareLaunchArgument('rgb_topic', default_value='/camera/image', description='RGB image topic.'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth_image', description='Depth image topic.'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/camera_info', description='Camera info topic.'),
        DeclareLaunchArgument('scan_topic', default_value='/scan', description='Laser scan topic.'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered', description='Odometry topic.'),

        # RGB-D synchronization node
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{
                'approx_sync': True, 
                'use_sim_time': use_sim_time,
                'queue_size': 10
            }],
            remappings=remappings),
        
        # SLAM mode (Mapping):
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),  # Delete previous database
            
        # Localization mode (if you have a pre-built map):
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory': 'False',
               'Mem/InitWMWithAllNodes': 'True'}],
            remappings=remappings),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("rtabmap_viz")),
            parameters=[parameters],
            remappings=remappings),
            
        # RViz Visualization
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("rviz")),
            arguments=[["-d"], [config_rviz]]),
    ])