# Copyright 2021 the Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    """
     * ndt_map_publisher
    """


    ndt_map_publisher_param = {
        'map_pcd_file': "/home/yamato/data/kashiwanoha/pointcloud_map.pcd",
        'map_yaml_file': "/home/yamato/data/kashiwanoha/pointcloud_map.yaml",
        'map_frame': 'map',
        'map_config.capacity': 2500000,
        'map_config.min_point.x': 3500.0,
        'map_config.min_point.y': 73000.0,
        'map_config.min_point.z': -100.0,
        'map_config.max_point.x': 4000.0,
        'map_config.max_point.y': 74000.0,
        'map_config.max_point.z': 100.0,
        'map_config.voxel_size.x': 2.0,
        'map_config.voxel_size.y': 2.0,
        'map_config.voxel_size.z': 2.0,
        'viz_map': True
    }

    ndt_map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        name='ndt_map_publisher_node',
        parameters=[ndt_map_publisher_param]
    )


    """
     * ndt_localizer
    """
    localization_launch_pkg_prefix = get_package_share_directory(
        'localization_launch')
    ndt_localizer_param_file = os.path.join(
        localization_launch_pkg_prefix, 'config/ndt_localizer.param.yaml')

    # Arguments
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )

    # Nodes
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/localization/util/downsample/pointcloud"),
            ("ndt_pose", "/localization/pose_estimator/pose_with_covariance"),
            # ("ndt_pose", "/localization/pose_estimator/pose_with_covariance_dummy"),
            ("observation_republish", "/localization/p2d_ndt_localizer_node/points_fused_viz"),
        ]
    )

    return LaunchDescription([
        ndt_map_publisher,
        ndt_localizer_param,
        ndt_localizer,
    ])
