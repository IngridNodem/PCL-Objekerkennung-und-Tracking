# launch/benchmark.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, TimerAction, SetEnvironmentVariable, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import launch

def generate_launch_description():
    ns_arg   = DeclareLaunchArgument('ns', default_value='bench')
    bag_arg  = DeclareLaunchArgument('bag', default_value='')
    rate_arg = DeclareLaunchArgument('rate', default_value='1.0')
    secs_arg = DeclareLaunchArgument('run_seconds', default_value='0')
    # Start playback at an offset (seconds) from the beginning of the bag.
    # 0 = start from the beginning (default)
    start_off_arg = DeclareLaunchArgument('start_offset', default_value='0')
    # Expose crop box Y-bounds so scripts can sweep them
    min_y_arg = DeclareLaunchArgument('min_y', default_value='-6.0')
    max_y_arg = DeclareLaunchArgument('max_y', default_value='6.0')
    rmw_arg  = DeclareLaunchArgument('rmw', default_value='')  # e.g. 'rmw_fastrtps_cpp' or 'rmw_cyclonedds_cpp'
    # Toggle simulated time usage for latency measurements
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')

    # Algo-Parameter
    voxel     = DeclareLaunchArgument('voxel_size', default_value='0.20')
    dist_thr  = DeclareLaunchArgument('distance_threshold', default_value='0.15')
    tol       = DeclareLaunchArgument('cluster_tolerance', default_value='0.50')
    minpts    = DeclareLaunchArgument('min_cluster_size', default_value='40')
    maxpts    = DeclareLaunchArgument('max_cluster_size', default_value='8000')
    maxcls    = DeclareLaunchArgument('max_clusters', default_value='200')
    # BBox-Typ: aabb oder obb
    bbox_type = DeclareLaunchArgument('bbox_type', default_value='aabb')
    gate_d    = DeclareLaunchArgument('gate_dist_max', default_value='4.0')
    min_hits  = DeclareLaunchArgument('min_hits', default_value='2')
    max_miss  = DeclareLaunchArgument('max_missed', default_value='10')

    ns   = LaunchConfiguration('ns')
    bag  = LaunchConfiguration('bag')
    rate = LaunchConfiguration('rate')

    env = [
        # Only override RMW if provided
        SetEnvironmentVariable('RMW_IMPLEMENTATION', LaunchConfiguration('rmw'),
                               condition=LaunchConfigurationNotEquals('rmw', '')),
        # Only set CycloneDDS-specific URI when CycloneDDS is selected
        SetEnvironmentVariable('CYCLONEDDS_URI',
            '<CycloneDDS><Domain><SharedMemory><Enable>true</Enable></SharedMemory></Domain></CycloneDDS>',
            condition=LaunchConfigurationEquals('rmw', 'rmw_cyclonedds_cpp'))
    ]

    play_bag = ExecuteProcess(
        cmd=['ros2','bag','play', bag, '--clock', '--rate', rate, '--start-offset', LaunchConfiguration('start_offset')],
        output='screen',
        condition=LaunchConfigurationNotEquals('bag', '')  # exécuter seulement si 'bag' n'est pas vide
    )

    pipeline = GroupAction([
        PushRosNamespace(ns),

        Node(package='ouster_cpp', executable='crop_box_node', name='crop_box_node_cpp', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'input_topic': '/ouster/points', 'output_topic': 'points_cropped',
                          # Elements in arrays must be plain values or Substitutions, not ParameterValue
                          'min_bound': [-10.0, LaunchConfiguration('min_y'), -3.0],
                          'max_bound': [ 10.0, LaunchConfiguration('max_y'),  5.0]}]),

        Node(package='ouster_cpp', executable='voxel_filter_node', name='voxel_filter_node_cpp', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'input_topic': 'points_cropped', 'output_topic': 'points_voxel',
                          'voxel_size': LaunchConfiguration('voxel_size')}]),

        Node(package='ouster_cpp', executable='ransac_ground_node', name='ransac_ground_node_cpp', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'input_topic': 'points_voxel', 'output_topic': 'obstacle_points',
                          'distance_threshold': LaunchConfiguration('distance_threshold'), 'max_iterations': 1000}]),

        Node(package='ouster_cpp', executable='cluster_extraction_node', name='cluster_extraction_node_cpp', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'input_topic': 'obstacle_points', 'marker_topic': 'detections_markers',
                          'detections_topic': 'detections_raw', 'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                          'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                          'max_cluster_size': LaunchConfiguration('max_cluster_size'),
                          'max_clusters': LaunchConfiguration('max_clusters'),
                          'bbox_type': LaunchConfiguration('bbox_type')}]),

        Node(package='ouster_cpp', executable='tracking_node', name='tracking_node_cpp', output='screen',
             parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'detections_topic': 'detections_raw',
                          'tracks_topic': 'tracks_raw', 'tracks_markers': 'tracks_markers',
                          'gate_dist_max': LaunchConfiguration('gate_dist_max'),
                          'max_missed': LaunchConfiguration('max_missed'),
                          'min_hits': LaunchConfiguration('min_hits')}]),

    ])

    timed_shutdown = TimerAction(
        period=LaunchConfiguration('run_seconds'),
        actions=[EmitEvent(event=launch.events.Shutdown())],
        condition=LaunchConfigurationNotEquals('run_seconds', '0')
    )

    return LaunchDescription([
        ns_arg, bag_arg, rate_arg, secs_arg, start_off_arg, min_y_arg, max_y_arg, rmw_arg, use_sim_time_arg,
        voxel, dist_thr, tol, minpts, maxpts, maxcls, bbox_type, gate_d, min_hits, max_miss,
        *env, play_bag, pipeline, timed_shutdown
    ])
