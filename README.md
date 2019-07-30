# MOCAP4ROS2

Driver VICON - ROS2

## Guide

Connect your computer/lapton to the same network as Vicon system is connected.

Run Nexus (Vicon software) and calibrate cameras (if required).

Launch the Vicon-ROS2 driver launcher:

` ` `
ros2 launch vicon2_bridge vicon.launch.py
` ` `

Check new topics where Vicon info is received in custom message format and TFs format.
