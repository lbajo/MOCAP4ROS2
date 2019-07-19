import os
import pathlib
import unittest

from launch import LaunchDescription
from launch import LaunchService
from launch_ros.substitutions import ExecutableInPackage
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='vicon2_bridge', node_executable='vicon2_bridge_main',
            node_name='vicon_node', output='screen',
            #parameters=[{'abc': 2}],
        )
        #executable = ExecutableInPackage(package='vicon2_bridge', executable='simpletest')
        #name = 'simpletest'
        #launch_ros.actions.ExecuteProcess(
        #    cmd=[executable, name,  '--stream_mode', "ClientPull",
        #                            '--datastream_hostport', "192.168.10.1:801",
        #                            '--tf_ref_frame_id', "world",
        #                            '--enable_tf_broadcast', "true",
        #                            output='screen']
        # )
    ])

"""
<launch>
	<!--node pkg="vicon_bridge" type="vicon_bridge" name="vicon" output="screen"-->
	<node pkg="vicon_bridge" type="marker_tf" name="vicon" output="screen">
		<param name="stream_mode" value="ClientPull" type="str" />
		<param name="datastream_hostport" value="192.168.10.1:801" type="str" />
		<!--param name="datastream_hostport" value="vicon:801" type="str" /-->
		<param name="tf_ref_frame_id" value="world" type="str" />
		<param name="enable_tf_broadcast" value="true" type="bool"/>
		</node>
	<!--node pkg="tf" type="static_transform_publisher" name="static_vicon" args="1 0 0 0 0 0 1 world vicon_marker 100"/-->
</launch>
"""
