# rmf_robot_demo
RMF robot project, involved using a UR10, dynamixel gripper, and AGV integration in demo showcase.
ROS1 and ROS2 in communication

UR10 arm Manipulation: uses ROS MOVEIT! in motino planning
Gripper: 


## Environment Setup
1) ROS installed


2) ur10_config Download
http://wiki.ros.org/ur10_moveit_config

* for kinetic:
	https://github.com/iron-ox/ur_modern_driver/tree/iron-kinetic-devel


## Run Demo Script
Robot’s config file (Planner type too: STOMP or OMPL)
> roslaunch ur10_rmf ur10_test.launch
Or 
> roslaunch ur5_moveit_config moveit_rviz.launch config:=true

Motion Planning script
> rosrun ur10_rmf python_moveit2.py



## Run On UR10 Robot hardware
config IP, create IP for UR10 in ubuntu connection, different port num exm: 192.168.88.222
Then Try to ping the connection 192.168.88.70 (robot’s)


