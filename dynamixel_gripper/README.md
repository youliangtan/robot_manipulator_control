# Dynamixel Servo Gripper

## Purpose

An end effector is the device at the end of a robotic arm, designed to interact with the environment. To provide for a low cost gripper solution for the underlying project, 2 [dynamixel motors](https://www.trossenrobotics.com/shared/images/PImages/R-903-0188-000-c.jpg) with claws affixed are attached alongside each other. This library (**built & tested with ROS kinetic, Ubuntu 16.04**) provides an easy-to-use ROS package to control the grip.

## Getting started

* Ensure dynamixel motors have the proper power supply & both servos have a different ID (you can change them by using a [GUI tool](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#gui)
* Install [dynamixel_sdk](http://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/download/#repository)
* Ensure you have download the packages for dynamixel_controllers; you can do it by running the command:
```
sudo apt-get install ros-kinetic-dynamixel-controllers
```
* Download & place this package in your_catkin_workspace/src; remember to overlay your workspace directory in ROS_PACKAGE_PATH
* Run the following commands:
```
catkin_make
roscd dynamixel_gripper/scripts
chmod +x getLoad.py
chmod +x GripperOpenClose.py
```
* To start using the ROS API:
```
roslaunch dynamixel_gripper gripper_manager.roslaunch
```

## ROS API

**Published Topics**
* /gripper/load (msg type: dynamixel_gripper/load_state)
  - Returns the current load in both servos
* /gripper/state (msg type: dynamixel_gripper/grip_state)
  - Returns 0 if gripper is open, and 1 if gripper is closed; contains other motor info as well

**Subscribed Topics**
* /gripper/command (msg type: std_msgs/Int32)
  - 0 - open, 1 - close

## Important points
* Gripper may get hot after prolonged use. Keep track and ensure its **temperature does not exceed 65 degrees celsius**. To track temperature, check published topic **/gripper/state**
* Certain parameters can be adjusted in 'tilt.yaml' & 'gripper_manager.launch' files, such as _port number, servo id search range, baudrate, joint velocity, open & close angles_
* Ensure the dynamixel port in use follows _"/dev/ttyUSB0"_
* Left dynamixel gripper servo should be labeled as ID 1, and right dynamixel gripper servo labeled as ID 2
