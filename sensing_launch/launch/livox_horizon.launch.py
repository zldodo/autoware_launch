
# Copyright 2020 Tier IV, Inc. All rights reserved.
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

import launch
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable
from launch.conditions import IfCondition


def get_vehicle_info(context):
    path = LaunchConfiguration('vehicle_param_file').perform(context)
    with open(path, 'r') as f:
        p = yaml.safe_load(f)['/**']['ros__parameters']
    p['vehicle_length'] = p['front_overhang'] + p['wheel_base'] + p['rear_overhang']
    p['vehicle_width'] = p['wheel_tread'] + p['left_overhang'] + p['right_overhang']
    p['min_longitudinal_offset'] = -p['rear_overhang']
    p['max_longitudinal_offset'] = p['front_overhang'] + p['wheel_base']
    p['min_lateral_offset'] = -(p['wheel_tread'] / 2.0 + p['right_overhang'])
    p['max_lateral_offset'] = p['wheel_tread'] / 2.0 + p['left_overhang']
    p['min_height_offset'] = 0.0
    p['max_height_offset'] = p['vehicle_height']
    return p


def launch_setup(context, *args, **kwargs):

    pkg = 'pointcloud_preprocessor'
    use_tag_filter = IfCondition(LaunchConfiguration("use_tag_filter")).evaluate(context)

    vehicle_info = get_vehicle_info(context)

    bd_code_param_path = LaunchConfiguration('bd_code_param_path').perform(context)
    with open(bd_code_param_path, 'r') as f:
        bd_code_param = yaml.safe_load(f)['/**']['ros__parameters']

    # livox driver
    livox_driver_component = ComposableNode(
        package='livox_ros2_driver',
        plugin='livox_ros::LivoxDriver',
        name='livox_driver',
        parameters=[
            {
                'xfe_format': LaunchConfiguration('xfe_format'),
                'multi_topic': LaunchConfiguration('multi_topic'),
                'data_src': LaunchConfiguration('data_src'),
                'publish_freq': LaunchConfiguration('publish_freq'),
                'output_data_type': LaunchConfiguration('output_type'),
                'lvx_file_path': LaunchConfiguration('lvx_file_path'),
                'user_config_path': LaunchConfiguration('user_config_path'),
                'frame_id': LaunchConfiguration('sensor_frame'),
                'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME',
                                                    default_value='False'),
            },
            bd_code_param,
        ]
    )

    # livox tag filter
    livox_tag_filter_component = ComposableNode(
        package="livox_tag_filter",
        plugin='livox_tag_filter::LivoxTagFilterNode',
        name="livox_tag_filter",
        remappings=[
            ('input', 'livox/lidar'),
            ('output', 'livox/tag_filtered/lidar'),
        ],
        parameters=[{
            'ignore_tags': [1, 2],
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }]
    )

    # set min range filter as a component
    crop_box_min_range_component = ComposableNode(
        package=pkg,
        plugin='pointcloud_preprocessor::CropBoxFilterComponent',
        name='crop_box_filter_min_range',
        remappings=[
            ('input', "livox/lidar" if use_tag_filter else "livox/tag_filtered/lidar"),
            ('output', 'min_range_cropped/pointcloud'),
        ],
        parameters=[{
            'input_frame': LaunchConfiguration('sensor_frame'),
            'output_frame': LaunchConfiguration('base_frame'),
            'min_x': 0.0,
            'max_x': 1.5,
            'min_y': -2.0,
            'max_y': 2.0,
            'min_z': -2.0,
            'max_z': 2.0,
            'negative': True,
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }]
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name='pointcloud_preprocessor_container',
        namespace='pointcloud_preprocessor',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            crop_box_min_range_component,
        ],
        output='screen',
        parameters=[{
            'use_sim_time': EnvironmentVariable(name='AW_ROS2_USE_SIM_TIME', default_value='False'),
        }],
    )

    livox_driver_loader = LoadComposableNodes(
        composable_node_descriptions=[livox_driver_component],
        target_container=container,
        condition=launch.conditions.IfCondition(LaunchConfiguration('launch_driver')),
    )

    livox_tag_filter_loader = LoadComposableNodes(
        composable_node_descriptions=[livox_tag_filter_component],
        target_container=container,
        condition=IfCondition(LaunchConfiguration("use_tag_filter")),
    )

    return [container, livox_driver_loader, livox_tag_filter_loader]


def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg('xfe_format', '0')
    add_launch_arg('multi_topic', '0')
    add_launch_arg('data_src', '0')
    add_launch_arg('publish_freq', '10.0')
    add_launch_arg('output_type', '0')
    add_launch_arg('lvx_file_path', 'livox_test.lvx')
    add_launch_arg('user_config_path', os.path.join(get_package_share_directory(
        "livox_ros2_driver"), "config/livox_lidar_config.json"))
    add_launch_arg('bd_code_param_path')
    add_launch_arg('launch_driver')
    add_launch_arg('base_frame', 'base_link')
    add_launch_arg('sensor_frame', 'livox_frame')
    add_launch_arg('use_tag_filter', 'true')
    add_launch_arg('vehicle_param_file')
    add_launch_arg('vehicle_mirror_param_file')

    return launch.LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
