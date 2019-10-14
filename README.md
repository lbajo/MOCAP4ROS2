# MOCAP4ROS2

This project provides support for ROS2 integration with Vicon cameras (MOCAP systems based on vision) and Technaid TechMCS IMUs (MOCAP systems based on motion sensors).

The project [MOCAP4ROS2](https://rosin-project.eu/ftp/mocap4ros2) is funded as a Focused Technical Project by [ROSIN](http://rosin-project.eu/).


<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" 
       alt="rosin_logo" height="60" >
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.  
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" 
     alt="eu_flag" height="45" align="left" >  

This project has received funding from the European Union’s Horizon 2020  
research and innovation programme under grant agreement no. 732287. 

***

## Guide

### Vicon

- Connect your computer/laptop to the same network as Vicon system is connected.
- Run Nexus (Vicon software) and calibrate cameras (if required).
- Launch the Vicon-ROS2 driver launcher: `ros2 launch vicon2_bridge vicon.launch.py`
- Check new topics where Vicon info is received in custom message format and TFs format.

### TechMCS IMUs

- Open `TechMCS_ROS2.sln` on Visual Studio 2017.
- Connect Tech MCS IMUs to your computer.
- Run the Visual Studio solution and check new topics where IMUs info is received.
