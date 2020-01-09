# Copyright 2019 Intelligent Robotics Lab
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
# Author: David Vargas Frutos <david.vargas@urjc.es>

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    project_dir = get_package_share_directory('vicon2_driver')
    params_file = LaunchConfiguration('params_file')

    #configured_params = RewrittenYaml(
    #        source_file=params_file)

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(project_dir, 'config', 'vicon_driver_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    start_vicon2_driver_cmd = Node(
        package='vicon2_driver',
        node_executable='vicon2_driver_main',
        node_name='vicon2_driver_node',
        parameters=[{'params_file': params_file}],
        #parameters=[get_package_share_directory('vicon2_driver')+'/config/vicon_driver_params.yaml'],
        output='screen')

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(start_vicon2_driver_cmd)

    return ld

    # return LaunchDescription([
    #     launch_ros.actions.Node(
    #         package='vicon2_driver',
    #         node_executable='vicon2_driver_main',
    #         node_name='vicon_node',
    #         output='screen',
    #         parameters=[{get_package_share_directory('vicon2_driver')+'/config/vicon_driver_params.yaml'}]
    #     )
    # ])

        # executable = ExecutableInPackage(package='vicon2_bridge', executable='simpletest')
        # name = 'simpletest'
        # launch_ros.actions.ExecuteProcess(
        #    cmd=[executable, name,  '--stream_mode', "ClientPull",
        #                            '--datastream_hostport', "192.168.10.1:801",
        #                            '--tf_ref_frame_id', "world",
        #                            '--enable_tf_broadcast', "true",
        #                            output='screen']
        # )
