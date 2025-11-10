from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rtabmap_launch'),
                    'launch',
                    'rtabmap.launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': 'true',
                
                'frame_id': 'base_footprint',
                'odom_frame_id': 'odom', 
                'map_frame_id': 'map',
                'publish_tf': 'true',
                'publish_tf_map': 'true',

                'rgb_topic': '/camera/image',
                'depth_topic': '/camera/depth_image',
                'camera_info_topic': '/camera/camera_info',
                'scan_topic': '/scan',
                'odom_topic': '/odom',  # Use raw Gazebo odometry

                'subscribe_odom_info': 'false',
                'visual_odometry': 'false', 
                'icp_odometry': 'false',

                'subscribe_rgbd': 'true',
                'subscribe_scan': 'true',
                'rgbd_sync': 'true',
                'approx_sync': 'true',
                'approx_rgbd_sync': 'true',
                'compressed': 'false', 
                
                # IMPROVED PARAMETERS FOR BETTER MAP QUALITY
                'args': '-d \
                    --Grid/3D true \
                    --Grid/FromDepth true \
                    --Grid/CellSize 0.05 \
                    --Grid/RangeMax 4.0 \
                    --cloud_output_voxel true \
                    --cloud_voxel_size 0.05 \
                    --Kp/MaxFeatures 1000 \
                    --Vis/MaxFeatures 1000 \
                    --Vis/MinInliers 15 \
                    --Mem/STMSize 100 \
                    --RGBD/LinearUpdate 0.5 \
                    --RGBD/AngularUpdate 0.5 \
                    --Reg/Force3DoF true \
                    --Mem/ImageCompression true \
                    --Mem/CompressionParallelized true',
                
                'rviz': 'true'
            }.items()
        ),
    ])