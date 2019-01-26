# Robot_Manipulator_Control
(DEVELOPING) robot manipulator project, involved in using a UR10, 'self-made' dynamixel gripper.
ROS1 and ROS2 will eventually be used in communication integration. Two ROS 1 packages are used here:

- **UR10 robot manipulator**: uses [ROS MOVEIT!](https://moveit.ros.org) in motion planning. Refer to `ur10_rmf` pkg.
- **Dynamixel Gripper**: Refer to the `README.md` in the package folder. Refer to 'dynamixel_gripper' pkg.

![alt text](/resources/ur10_with_gripper.png?)

Here, step-by-step instructions are listed here to guide user on how to setup the environment to: (1) Run it on Rviz  (2) Run it on a real UR10 Robot!!!!! Have fun people!!

## Environment Setup
1) Install ROS
   Refer to here: http://wiki.ros.org/ROS/Installation

2) ur10_moveit_config Download
According to http://wiki.ros.org/ur10_moveit_config
```
sudo apt-get install ros-kinetic-ur10-moveit-config

```

3) MoveIt Install 
```
sudo apt-get install ros-indigo-moveit
```

3) Install Package
```
catkin_make
roscd ur10_rmf/scripts
chmod +x robot_manipulator_control.py
source ~/xxx/devel/setup.bash
```

#### ** 4) If Interfacing with UR10 hardware
4.1) ur_msgs Install
```
sudo apt-get install ros-kinetic-ur-msgs    # to interface with hardware
```

4.2) 'ur_modern_driver' build
```
git clone https://github.com/iron-ox/ur_modern_driver
git checkout iron-kinetic-devel` # if using kenetic
catkin_make --pkg ur_modern_driver
```

4.3) Disable mock joint state pub
```
roscd ur10_rmf
nano launch/ur10_test.launch
```
- comment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run with UR10 hardware


## Run Demo Script

#### 1) Run On Rviz
Robot’s config file
`roslaunch ur10_rmf ur10_test.launch` or default `roslaunch ur10_moveit_config demo.launch`

#### 2) Motion Planning Python Script
To program a series of motion , edit the yaml file at `config/motion_config.yaml`. 

Once done, run the script to visualize the motion control.
```
rosrun ur10_rmf robot_manipulator_control.py
```

#### Note
- uncomment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run without UR10 hardware 
- Change planning between: STOMP, CHOMP, OMPL
- If select STOMP planner, build package from [industrial moveit](https://github.com/ros-industrial/industrial_moveit)
- In `motion_config.yaml` file, edit `enable_gripper` to `False` to disable pub sub to gripper's topic



## Run On UR10 hardware

#### Config on UR10 GUI
config IP, create IP for UR10 in ubuntu connection, different ip num for the 4th unmasked session, `192.168.88.XXX` (e.g: `192.168.88.222`)
Then Try to ping the connection 192.168.88.70 (depends on robot’s ip)


#### 1) Bring up connection between PC and UR10
```
roslaunch ur_modern_driver ur10_bringup.launch robot_ip:=192.168.88.70 [reverse_port:=REVERSE_PORT]
```
**IP is robot’s IP

#### 2) Visualize on Rviz
```
roslaunch ur10_rmf ur10_test.launch
```

#### 3) Enable Moveit Execution on hardware
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true
```

#### 4) Run Script
```
rosrun ur10_rmf robot_manipulator_control.py
rostopic pub /ur10/motion_group_id std_msgs/String "INPUT" #INPUT: G1, G2... 

```

# Code Explanation

### ManipulatorControl Class
Class `ManipulatorControl` simplfied the use of typing code to control the robot manipulator. This helps user to create a series of motion just by edit the `motion_config.yaml` file. 3 useful functions in this class are:

- **execute_all_motion_group()**, 
- **execute_motion_group(string motion_group_id)**  return True/False, True: success, False: fail
- **execute_motion(string motion_id)**  return True/False, True: success, False: fail
- **execute_motion_group_service()**

### ArmManipulation Class
This class directly interact with the ROS `moveit` package. 

- **go_to_joint_state(joint_goal, time_factor)** return `bool` (success anot)
- **go_to_pose_goal(self, pose_list, time_factor )** return `bool` (success anot)
- **plan_cartesian_path(self, motion_list, time_factor)** return `obj`, `float`  (planned trajectory, success fraction, 1.0 is successful planning )
- **execute_plan(self, plan)** return `bool` (success anot)

### Pub Sub for execute_group_service()
Use `ur10.execute_motion_group_service()` to start ros service, which request group_id to user.

**Sub**: 
- /ur10/motion_group_id: Group ID (string)

**Pub**: 
- /ur10/manipulator_state: State of arm and gripper (custom msg)
- /ur10/rm_bridge_state: same as above's state, temp solution to feed to ros bridge (float32_array)



### More Notes
- Kinetic will have prob on using `ur_modern_driver`, so need to find fork copy:
	https://github.com/iron-ox/ur_modern_driver/tree/iron-kinetic-devel
- Comment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run with UR10 hardware 
- refer to `dynamixel_gripper` package to run gripper with ur10
- If wanna run with the gripper, pls refer to the package `readme.md` to run the launch file: `roslaunch dynamixel_gripper gripper_manager.roslaunch`
- ur10 control is a higer level control of dynamixel gripper
- Edit `enable_gripper` in .yaml file to `True` to enable usage of gripper
- In the yaml file, the hierachy of each is: `motion_group` > `motion` > `cartesian_motion`.
- Use execute_motion_group_service to check printout of `current 6 joints` and `current eef pose`, this helps in configuring the motion yaml
