from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ========================
    # --- Parameter-Setup ---
    # ========================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace    = LaunchConfiguration('namespace', default='pipeline')
    cloud_in     = LaunchConfiguration('cloud_in',     default='/ouster/points')
    cloud_out    = LaunchConfiguration('cloud_out',    default='obstacle_points')  # relativ → /pipeline/obstacle_points
    frame_id     = LaunchConfiguration('frame_id',     default='base_link')
    lidar_frame  = LaunchConfiguration('lidar_frame',  default='os_lidar')
    map_frame    = LaunchConfiguration('map_frame',    default='map')
    db_path      = LaunchConfiguration('db_path',      default='/tmp/rtabmap_mapping.db')

    # =====================================================
    # --- 0) Statische TF (base_link -> os_lidar) ---
    # =====================================================
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_tf',
        # x y z roll pitch yaw parent child
        arguments=['0.0','0.0','0.0','0.0','0.0','0.0', frame_id, lidar_frame],
        output='screen'
    )

    # =====================================================
    # --- 1) Ouster-Vorverarbeitung (ouster_cpp) ---
    # =====================================================
    # → Filtert die LiDAR-Daten (CropBox, Voxel, RANSAC)
    preproc_node = Node(
        package='ouster_cpp',
        executable='ransac_ground_node',
        name='ransac_ground_node_cpp',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'distance_threshold': 0.25,  # max. Abstand zur Bodenebene [m]
            'max_iterations': 1000,      # RANSAC-Iterationen
        }],
        remappings=[
            ('points_voxel', cloud_in),   # Eingabe (vom vorherigen Filter oder /ouster/points)
            ('obstacle_points', cloud_out)  # Ausgabe → wird /pipeline/obstacle_points
        ]
    )


        # 1bis) Odométrie ICP (produit /odom et TF odom->base_link)
    icp_odom = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,             # "base_link"
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'subscribe_scan_cloud': True,     
            'Reg/Strategy': '1',
            'Icp/VoxelSize': '0.10',
            'Icp/MaxCorrespondenceDistance': '0.5',
            'Icp/PointToPlane': 'true'
        }],
        remappings=[
            ('scan_cloud', cloud_out)      # /pipeline/obstacle_points -> entrée ICP
        ]
    )


    # =====================================================
    # --- 2) RTAB-Map (SLAM) ---
    # =====================================================
    # Arbeitet direkt mit den vorverarbeiteten Punkten
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            # ---- Wrapper ----
            {
                'use_sim_time': True,
                'frame_id': frame_id,
                'map_frame_id': map_frame,
                'odom_frame_id': frame_id,   # Kein echtes Odom → base_link als Referenz
                # Sensor-Abos
                'subscribe_rgbd': False,
                'subscribe_depth': False,
                'subscribe_rgb': False,
                'subscribe_stereo': False,
                'subscribe_scan': False,
                'subscribe_scan_cloud': True,
                # Synchronisation
                'approx_sync': True,
                'queue_size': 100,
                'topic_queue_size': 50,
                'sync_queue_size': 100,
                'approx_sync_max_interval': 0.20,
            },
            # ---- RTAB-Map Core (Strings!) ----
            {
                'Reg/Strategy': '1',                # ICP
                'RGBD/LoopClosureReextract': 'true',
                'Mem/IncrementalMemory': 'true',    # Mapping-Modus
                'Grid/CellSize': '0.10',            # Kartenauflösung [m]
                'database_path': db_path
            }
        ],
        remappings=[
            ('scan_cloud', cloud_out)  # /pipeline/obstacle_points
        ]
    )

    # =====================================================
    # --- Rückgabe der LaunchDescription ---
    # =====================================================
    return LaunchDescription([
        # ---- Globale Argumente ----
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='pipeline'),
        DeclareLaunchArgument('cloud_in',  default_value='/ouster/points'),
        DeclareLaunchArgument('cloud_out', default_value='obstacle_points'),
        DeclareLaunchArgument('frame_id',  default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='os_lidar'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('db_path',  default_value='/tmp/rtabmap_mapping.db'),

        # ---- Namespace anwenden ----
        PushRosNamespace(namespace),

        # ---- Nodes starten ----
        static_tf,
        preproc_node,
        icp_odom,
        rtabmap
    ])
