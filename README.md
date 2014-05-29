# ArDroneControl: A ROS package for state estimation and control for ARDrone

ArDroneControl is a ROS package for state estimation and pose control of a Parrot AR-Drone quadcopter.

"ArDroneControl" is a [ROS](http://ros.org/ "Robot Operating System") package for state estimate and control of a [Parrot AR-Drone](http://ardrone2.parrot.com/) quadrocopter. This package is to be used with [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy/ "AutonomyLab ardrone_autonomy"), [artoolkit](wiki.ros.org/artoolkit), and [tum_ardrone](wiki.ros.org/tum_ardrone). 

this package has the following nodes to be used:

localization/slam : this node uses the ardrone bottom camera images to make a map of artookit markers and localize the drone pose using a Kalman Filter.

position_control: this node implements a PD controler to control the drone's pose (position and yaw angle). the pose feedback can be made using the slam/localization nodes or tum_ardrone's PTAM SLAM.

planner: This node generates setpoint to the position_control node. One can load a predefined pose path and command the drone to follow it. the path is relative to an start frame witch can be at the origin (x=0,y=0,z=0 in map coordinates) or any other position.

tum_interface: this node is used to provide pose feedback using tum_ardrone's PTAM SLAM.

keyinput: this node passes keyboard inputs to the other nodes. select the terminal window and press the following letters:
"L" - Lock slam mapping and save map.
"U" - Unlock mapping
"R" - Reset map
"G" - Set start frame as current pose, enable control and hold current pose.
"0" - Set start frame as origin, enable control and hold current pose.
"P" - Start following loaded path relative to startframe.
"M" - Disable control and put drone back to manual.


PS.: this package currently uses tum_ardrone's GUI to control drone in manual mode. Even when the source of control is set to "none" in drone_gui it keeps sending control signals. So to use the position_control node you have to close drone_gui.
