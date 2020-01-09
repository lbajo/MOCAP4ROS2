#Prerequisites

## Installing Vicon DataStream SDK

Get the official binaries released in the official download page [here](https://www.vicon.com/software/datastream-sdk/?section=downloads).

You can check the documentation [here](https://docs.vicon.com/spaces/viewspace.action?key=DSSDK19).

Copy all the libraries to /usr/local/lib and the headers to /usr/local/include/ViconDataStreamSDK/.

## Guide

- Connect your computer/laptop to the same network as Vicon system is connected.

- Run Nexus (Vicon software) and calibrate cameras (if required).

- Launch the Vicon-ROS2 driver launcher: `ros2 launch vicon2_bridge vicon.launch.py`

- Check new topics where Vicon info is received in custom message format and TFs format.
