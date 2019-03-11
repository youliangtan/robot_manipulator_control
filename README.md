# Robot_Manipulator_Control
(DEVELOPING) robot manipulator project, involved in using a UR10, 'self-made' dynamixel gripper.
ROS1 and ROS2 will eventually be used in communication integration. Two ROS 1 packages are used here:

- **UR10 robot manipulator**: uses [ROS MOVEIT!](https://moveit.ros.org) in motion planning. Refer to `ur10_rmf` pkg.
- **Dynamixel Gripper**: Refer to the `README.md` in the package folder. Refer to 'dynamixel_gripper' pkg.

Used with Dynamic cartesian planning (optional)
- **Urg_node**: Optional hardware intergration with hokoyu lidar, package is [Here](https://github.com/ros-drivers/urg_node)
- **Object Pose Estimation**: Optional detection of target object's pose respective to lidar, package is [Here](https://github.com/tanyouliang95/object_pose_estimation)

![alt text](/resources/ur10_with_gripper.png?)

Here, step-by-step instructions are listed here to guide user on how to setup the environment to: (1) Run it on Rviz  (2) Run it on a real UR10 Robot!!!!! Have fun people!!

---

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

4.2) `ur_modern_driver` build
```
git clone https://github.com/ros-industrial/ur_modern_driver/
git checkout kinetic-devel`             # if using kenetic
catkin_make --pkg ur_modern_driver      # here apt-get all relevent dependencies of ur_modern_driver
```

---

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

#### 3) Optional Dynamic Planning on target (with hokoyu)
```
roslaunch urg_node urg_lidar.launch                         # pls configure hokoyu's ip
rosrun object_pose_estimation object_pose_estimation_ros
```

#### Notes
- uncomment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run without UR10 hardware 
- Change planning between: STOMP, CHOMP, OMPL
- If select STOMP planner, build package from [industrial moveit](https://github.com/ros-industrial/industrial_moveit)


---


## Run On UR10 hardware
Here, Hokoyu Lidar is used for pose estimation of the target object. Refer to `object_pose_estimation` ros pkg for reference.

#### Config on UR10 GUI
config IP, create IP for UR10 in ubuntu connection, different ip num for the 4th unmasked session, `192.168.88.XXX` (e.g: `192.168.88.222`)
Then try to ping the connection 192.168.88.70 (depends on robot’s ip)

#### 1) Launch MoveGroup, Rviz, Urg_node, Ur10 Hardware BringUp
```
roslaunch ur10_rmf ur10_hardware.launch
```
Here, you will be able to see the current ur10 hardware pose on Rviz. If dynamic cartesian planning is not used, user can comment out `urg_node`, `object_pose_estimation` pkg. 


#### 2) Enable Moveit Execution on hardware
```
roslaunch ur10_moveit_config ur10_moveit_planning_execution.launch limited:=true
```
* Steps 1 and 2 combined: `roslaunch ur10_rmf ur10_hardware_combined.launch`


#### 3) Run Script
```
rosrun ur10_rmf robot_manipulator_control.py
rostopic pub /ur10/motion_group_id std_msgs/String "INPUT" #INPUT: G1, G2... 

```

---

## Code Explanation

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
Use `ur10.execute_motion_group_service()` to start ros service, which request group_id to user. This will activate ros pub sub mentioned below:

- /ur10/motion_group_id: Group ID (string)  **Sub**
- /ur10/manipulator_state: State of arm and gripper (custom msg) **Pub**
- /ur10/rm_bridge_state: same as above's state, temp solution to feed to ros bridge (float32_array) **Pub**

### Other Pub Sub Being used by service and non-service function
- /gripper/state: gripper status (grip_state msg) **Sub**
- /gripper/command: command gripper on ros1 (Int32) **Pub**
- /ur10/target_pose: 2D Pose from pose estimation (Pose2D) msg  **Sub**

---

## Additional Notes
- Comment `joint_state_publisher` node in `ur10_test.launch` will enable rviz ur10 to run with UR10 hardware 
- refer to `dynamixel_gripper` package to run gripper with ur10
- If wanna run with the gripper, pls refer to the package `readme.md` to run the launch file: `roslaunch dynamixel_gripper gripper_manager.roslaunch`
- ur10 control is a higher level control of dynamixel gripper
- To know current pose and joint angle, check printout after each motion

### Understand motion_config.yaml
All defination and setting of motion is configure on `config/motion_config.yaml`. User just need to change the config file to configure each request motion.

- 4 types of motion: `cartesian`, `joint_goal`, `pose_goal`, `2d_dynamic_cartesian`
- 2 types of gripper motion: `eef_grip_obj`, `eef_release_obj`
- Edit `enable_gripper` in .yaml file to `True` to enable usage of gripper
- In the yaml file, the hierachy of each is: `motion_group` > `motion` > `cartesian_motion`.
- Use execute_motion_group_service to check printout of `current 6 joints` and `current eef pose`, this helps in configuring the motion yaml
- adding a Coefficeient, e.g. '3', '-2', on each motion_id or cartesian_motion_id is supported in the yaml file.

### Working with Pose Estimation
- To have dynamic cartesian planning, use `2d_dynamic_cartesian` in yaml file motion type. 
- Pose input of the target_pose is via ROSTOPIC `/ur10/target_pose` with [x, y, theta] info 
- RosMsg for pose is from a pose estimation node, e.g: lidar point cloud pose estimation.
- input 2D info of `fix_laser_pose`, `target`, `tolerance`
- [Here](https://github.com/tanyouliang95/object_pose_estimation) is my own lidar pose estimation ros node