# launch/ouster_pipeline.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # --- Launch-Argumente (vom Nutzer konfigurierbar) ---
    ns_arg        = DeclareLaunchArgument('ns', default_value='pipeline')            # Namespace für alle Nodes
    in_arg        = DeclareLaunchArgument('input_topic', default_value='/ouster/points')  # Eingangsdaten vom Ouster-LiDAR
    # Toggle simulated time for latency measurement campaigns
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Substitutionen
    ns           = LaunchConfiguration('ns')
    input_topic  = LaunchConfiguration('input_topic')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        ns_arg, in_arg, use_sim_time_arg,
        GroupAction([
            PushRosNamespace(ns),  # alle Nodes laufen unter /<ns>

            # --- Crop-Box-Filter (AABB) ---
            Node(
                package='ouster_cpp',
                executable='crop_box_node',
                name='crop_box_node_cpp',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic':  input_topic,        # absolut: bleibt /ouster/points
                    'output_topic': 'points_cropped',   # relativ: -> /<ns>/points_cropped
                    # Arbeitsvolumen (X,Y,Z) in Metern
                    'min_bound': [-10.0, -6.0, -3.0],
                    'max_bound': [ 10.0, 6.0, 5.0],
                }]
            ),

            # --- Voxel-Filter (Downsampling) ---
            Node(
                package='ouster_cpp',
                executable='voxel_filter_node',
                name='voxel_filter_node_cpp',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic':  'points_cropped',   # -> /<ns>/points_cropped
                    'output_topic': 'points_voxel',     # -> /<ns>/points_voxel
                    'voxel_size':   0.2,               # Voxelgröße [m]
                }]
            ),

            # --- RANSAC-Bodensegmentierung ---
            Node(
                package='ouster_cpp',
                executable='ransac_ground_node',
                name='ransac_ground_node_cpp',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic':       'points_voxel',     # -> /<ns>/points_voxel
                    'output_topic':      'obstacle_points',  # -> /<ns>/obstacle_points
                    'distance_threshold': 0.15,              # max. Abstand zur Bodenebene [m]
                    'max_iterations':     1000,              # RANSAC-Iterationen
                }]
            ),

            # --- Clustering + Bounding Boxes  ---
            Node(
                package='ouster_cpp',
                executable='cluster_extraction_node',
                name='cluster_extraction_node_cpp',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'input_topic':       'obstacle_points',     # -> /<ns>/obstacle_points
                    'marker_topic':      'detections_markers',  # -> /<ns>/detections_markers (RViz)
                    'detections_topic':  'detections_raw',      # -> /<ns>/detections_raw (für Tracker)

                    # Clustering-Tuning:
                    'cluster_tolerance':  0.5,   # Punktabstand [m] innerhalb eines Clusters
                    'min_cluster_size':   40,    # min. Punkte/Cluster (Rauschen unterdrücken)
                    'max_cluster_size':   8000,  # max. Punkte/Cluster
                    'max_clusters':       200,   # Obergrenze pro Frame
                }]
            ),

            # --- Tracking-Node (SORT-artig, robuste Orientierung + Klassifikation) ---
            Node(
                package='ouster_cpp',
                executable='tracking_node',
                name='tracking_node_cpp',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'detections_topic': 'detections_raw',   # Eingang vom Clustering
                    'tracks_topic':     'tracks_raw',       # Ausgabe (für weitere Verarbeitung)
                    'tracks_markers':   'tracks_markers',   # Marker für RViz

                    # Stabileres Tracking:
                    'gate_dist_max':    4.0,   # max. Assoziationsdistanz [m]
                    'max_missed':       10,    # Track-Überlebensdauer ohne Match (Frames)
                    'min_hits':         2,     # Bestätigungsschwelle (Frames)
                }]
            ),

        ])
    ])
