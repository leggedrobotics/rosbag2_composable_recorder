# -----------------------------------------------------------------------------
# Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
import os
import yaml

def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)
    
    
def launch_setup(context, *args, **kwargs):
    """Create composable node."""
    # Load configurations from YAML files
    camera_config = load_yaml("/data/workspaces/isaac_ros-dev/src/rosbag2_composable_recorder/config/cameras_config.yaml")
    ls = []
    for config in camera_config["cameras"]:            
        # TODO check if mt is needed  
        n = config["name"]

        topic_name = f"{config['namespace']}{config['name']}/image_raw"
        print("Current recording setting for images:", topic_name)
        ls.append( ComposableNodeContainer(
                name=f'container_{config["name"]}',
                namespace=config["namespace"],
                package='rclcpp_components',
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package="v4l2_camera",
                        plugin="v4l2_camera::V4L2Camera",
                        name=config["name"],
                        namespace=config["namespace"],
                        remappings=[
                            ("image_raw", f"{config['name']}/image_raw"),
                            ("image_raw/theora", f"{config['name']}/image_raw/theora"),
                            ("camera_info", f"{config['name']}/camera_info"),
                            ("image_raw/compressed", f"{config['name']}/image_raw/compressed"),
                        ],
                        parameters=[
                            {
                                "video_device": config["video_device"],
                                "hdr_enable": config["hdr_enable"],
                                "frame_id":  config["frame_id"],
                                "camera_info_url": config["camera_info_url"],
                                "use_image_transport": True,
                                "output_encoding": "rgb8",
                                "use_kernel_buffer_ts": True,
                                "use_sensor_data_qos": False,
                                "disable_pub_plugins": ["image_transport/compressedDepth"],  # Disabling the compressedDepth plugin
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package='rosbag2_composable_recorder',
                        plugin='rosbag2_composable_recorder::ComposableRecorder',
                        name="recorder_" + n,
                        namespace=config["namespace"],
                        # set topics etc here
                        parameters=[{   'storage_id': 'mcap',
                                        'record_all': False,
                                        'disable_discovery': False,
                                        'serialization_format': 'cdr',
                                        'start_recording_immediately': False,
                                        'bag_path': "/data/",
                                        'bag_name': f"_jetson_{config['name']}"
                                        }],
                        remappings=[],
                        extra_arguments=[{'use_intra_process_comms': True}],
                    ),               
                ],
                output='screen',
        ))
    return ls


def generate_launch_description():
    """Create composable node by calling opaque function.
    This creates a context such that string arguments can be extracted
    """
    launch_arguments = [
    ]

    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )