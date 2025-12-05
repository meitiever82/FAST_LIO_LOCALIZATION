#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fast_lio_localization')

    # Declare launch arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    map_path_arg = DeclareLaunchArgument(
        'map',
        default_value='/home/steve/Documents/Datasets/CASBOT/103/global_map.ply',
        description='Path to the prior PCD map file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock (true for rosbag with --clock)'
    )

    # Include airy_tf_tree.launch.py for TF configuration
    tf_tree_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'airy_tf_tree.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    # FastLIO Localization node
    fastlio_node = Node(
        package='fast_lio_localization',
        executable='fastlio_mapping',
        name='laserMapping',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'airy.yaml'),
            {
                # Node parameters
                'feature_extract_enable': False,
                'point_filter_num': 3,
                'max_iteration': 3,
                'filter_size_surf': 0.5,
                'filter_size_map': 0.5,
                'cube_side_length': 1000.0,
                'runtime_pos_log_enable': False,
                # Map path for localization
                'common.path_pcd': LaunchConfiguration('map'),
                # Set update_tree_frame to -1 for pure localization (no map update)
                'common.update_tree_frame': -1,
                # Use sim time
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz_cfg', 'loam_livox.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        # Launch arguments
        rviz_arg,
        map_path_arg,
        use_sim_time_arg,
        # TF tree (from airy_tf_tree.launch.py)
        tf_tree_launch,
        # Nodes
        fastlio_node,
        rviz_node,
    ])
