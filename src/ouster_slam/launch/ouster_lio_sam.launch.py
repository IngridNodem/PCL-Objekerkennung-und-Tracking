"""Launch file for running LIO-SAM SLAM with the Ouster processing pipeline."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    """Generate the launch description for the LIO-SAM based SLAM pipeline."""
    pkg_share = get_package_share_directory('ouster_slam')
    default_config = PathJoinSubstitution([
        pkg_share,
        'config',
        'lio_sam_ouster.yaml',
    ])

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    cloud_in = LaunchConfiguration('cloud_in')
    imu_topic = LaunchConfiguration('imu_topic')
    frame_id = LaunchConfiguration('frame_id')
    lidar_frame = LaunchConfiguration('lidar_frame')
    map_frame = LaunchConfiguration('map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    config = LaunchConfiguration('config')

    # Static transform base_link -> os_lidar
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', frame_id, lidar_frame],
        output='screen',
    )

    # LIO-SAM mapping node
    lio_sam_params = ParameterFile(config, allow_substs=True)
    lio_sam_node = Node(
        package='lio_sam',
        executable='lio_sam_mapping',
        name='lio_sam_mapping',
        output='screen',
        parameters=[
            lio_sam_params,
            {
                'use_sim_time': use_sim_time,
                'pointCloudTopic': cloud_in,
                'imuTopic': imu_topic,
                'mapFrame': map_frame,
                'odomFrame': odom_frame,
                'baselinkFrame': frame_id,
                'lidarFrame': lidar_frame,
            },
        ],
        remappings=[
            ('/lio_sam/imu/data', imu_topic),
            ('/imu/data', imu_topic),
            ('/imu', imu_topic),
            ('/points_raw', cloud_in),
            ('/velodyne_points', cloud_in),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='lio_sam'),
        DeclareLaunchArgument('cloud_in', default_value='/ouster/points'),
        DeclareLaunchArgument('imu_topic', default_value='/ouster/imu'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='os_lidar'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('config', default_value=default_config),

        PushRosNamespace(namespace),
        static_tf,
        lio_sam_node,
    ])
