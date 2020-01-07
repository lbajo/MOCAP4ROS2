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

from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='vicon2_driver', node_executable='vicon2_driver_main',
            node_name='vicon_driver_node', output='screen',
            # parameters=[{'abc': 2}],
        )
        # executable = ExecutableInPackage(package='vicon2_bridge', executable='simpletest')
        # name = 'simpletest'
        # launch_ros.actions.ExecuteProcess(
        #    cmd=[executable, name,  '--stream_mode', "ClientPull",
        #                            '--datastream_hostport', "192.168.10.1:801",
        #                            '--tf_ref_frame_id', "world",
        #                            '--enable_tf_broadcast', "true",
        #                            output='screen']
        # )
    ])
