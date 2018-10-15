# rmf_robot_demo
RMF robot project, involved using a UR10, dynamixel gripper, and AGV integration in demo showcase.
ROS1 and ROS2 in communication

UR10 arm Manipulation: uses [ROS MOVEIT!](https://moveit.ros.org) in motion planning

Dynamixel Gripper: *TBC, refer to fork copy in my repos*


## Environment Setup
1) Install ROS
   Refer to here: http://wiki.ros.org/ROS/Installation

2) ur10_moveit_config Download
According to http://wiki.ros.org/ur10_moveit_config
```
sudo apt-get install ros-kinetic-ur10-moveit-config
````

3) MoveIt Install 
```
sudo apt-get install ros-indigo-moveit
````
* for kinetic:
	https://github.com/iron-ox/ur_modern_driver/tree/iron-kinetic-devel

3) Install Package
```
catkin_make
roscd ur10_rmf/scripts
chmod +x python_moveit.py
````

## Run Demo Script

#### 1) Run On Rviz
Robot’s config file
`roslaunch ur10_rmf ur10_test.launch` or default `roslaunch ur10_moveit_config demo.launch`

#### 2) Motion Planning Python Script
```
rosrun ur10_rmf python_moveit2.py
````

### Note
- uncomment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run without UR10 hardware 
- Change planning between: STOMP, CHOMP, OMPL
- If select STOMP planner, build package from [industrial moveit](https://github.com/ros-industrial/industrial_moveit)
- Edit `enable_gripper` to 0 to disable pub sub to gripper's topic

## Run On UR10 Robot hardware
#### Config on UR10 GUI
config IP, create IP for UR10 in ubuntu connection, different port num, e.g: 192.168.88.222
Then Try to ping the connection 192.168.88.70 (robot’s)

#### 1) Bring up connection between PC and UR10
```
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.88.70 [reverse_port:=REVERSE_PORT]
````
**IP is robot’s IP

#### 2) Visualize on Rviz
```
roslaunch ur10_rmf ur10_test.launch
````

#### 3) Enable Moveit Execution on hardware
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true
````

#### 4) Run Script
```
rosrun ur10_rmf python_moveit2.py
````
### Note
- Kinetic will have prob on using ur_modern_driver, so need to find fork copy:
	https://github.com/iron-ox/ur_modern_driver/tree/iron-kinetic-devel
- Comment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run with UR10 hardware 
- Edit `enable_gripper` to 1 to enable usage of gripper
- refer to `dynamixel_gripper` package to run gripper with ur10
