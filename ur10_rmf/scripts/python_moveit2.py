#!/usr/bin/env python

# Open: 0, close: 1

import sys
import copy
import rospy
from time import sleep

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from arm_Manipulation import armManipulation

from math import pi
from std_msgs.msg import String
from std_msgs.msg import Int32
from dynamixel_gripper.msg  import grip_state

from moveit_commander.conversions import pose_to_list


rospy.init_node('ur10_motionPlanning', anonymous=True)
pub = rospy.Publisher('gripper/command', Int32, queue_size=10)

gripper_state = 0
rate = rospy.Rate(5) # 5hz

enable_gripper = 1


def gripperState_callback(data): 
  global gripper_state
  # rospy.loginfo( "I heard gripper state is {}".format(data.gripper_state))
  gripper_state = data.gripper_state
    
    
def is_gripperOpen(): 

  if (enable_gripper == 1):
    global gripper_state
    gripper_command = 0       # command to open gripper
    rospy.loginfo(gripper_command) #printout
    pub.publish(gripper_command)

    # wait until get state is open
    while (gripper_state == 1):
      print "gripper still close... with state".format(gripper_state)
      rate.sleep()


def is_gripperClose():

  if (enable_gripper == 1):
    global gripper_state
    gripper_command = 1       #command to close gripper
    rospy.loginfo(gripper_command)
    pub.publish(gripper_command)

    # wait until get state is close
    while (gripper_state == 0):
      print "gripper still open... with state {} ".format(gripper_state)
      rate.sleep()


def gripper_InOut_payLoad():

  print "============ Press `Enter` to put hand into payload ..."
  raw_input()
  cartesian_plan, planned_fraction = ur10.plan_cartesian_payload_path(scale=2)
  print " # Planned fraction: {} ".format(planned_fraction)
    
  if (planned_fraction == 1.0):
    ur10.execute_plan(cartesian_plan)

  is_gripperOpen()

  print "============ Press `Enter` to detach and remove the box from the Panda robot n scene..."
  raw_input()
  ur10.detach_box()
  ur10.remove_box()

  print "============ Press `Enter` to put hand out of payload ..."
  raw_input()
  cartesian_plan, planned_fraction = ur10.plan_cartesian_payload_path(scale=-2)
  print " # Planned fraction: {} ".format(planned_fraction)
  
  if (planned_fraction == 1.0):
    ur10.execute_plan(cartesian_plan)



## =============================================== MAIN ==============================================================


def main():

  rospy.Subscriber("gripper/state", grip_state, gripperState_callback)

  try:

    print "============ Press `Enter` to execute a movement to place beverage joint state goal ..."
    raw_input()
    place_joint_goal = [0, -pi/4, pi/2, -pi/4, pi/2, -pi/4]
    print " # Success? ", ur10.go_to_joint_state(place_joint_goal)
    is_gripperOpen()

    print "============ Press `Enter` to execute a movement to rest joint state goal ..."
    raw_input()
    rest_joint_goal = [0, -2*pi/5, pi/2+0.2, -pi/4 - 2*pi/5 + pi/2, pi/2, -pi/4]
    print " # Success? ", ur10.go_to_joint_state(rest_joint_goal)
    
    print "============ Press `Enter` to plan and display a Cartesian path ..."
    raw_input()
    cartesian_plan, planned_fraction = ur10.plan_cartesian_picking_path(scale=2)
    print " # Planned fraction: {} ".format(planned_fraction)

    if (planned_fraction == 1.0):
      ur10.execute_plan(cartesian_plan)

    print "============ Press `Enter` to add obj to scene and attach to robot ..."
    raw_input()
    ur10.add_box()
    ur10.attach_box()

    # Grip beveerage on rack
    is_gripperClose()

    print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
    raw_input()
    cartesian_plan, planned_fraction = ur10.plan_cartesian_placing_path(scale=2)
    print " # Planned fraction: {} ".format(planned_fraction)

    if (planned_fraction == 1.0):
      print ur10.execute_plan(cartesian_plan)

    print "============ Press `Enter` to execute a movement to place beverage joint state goal ..."
    raw_input()
    place_joint_goal = [0, -pi/4, pi/2, -pi/4, pi/2, -pi/4]
    print " # Success? ", ur10.go_to_joint_state(place_joint_goal)

    gripper_InOut_payLoad()

    print "============ Python demo complete! ========="


  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return


if __name__ == '__main__':
  
  print "============ Begin the python moveit script by setting up the moveit_commander (press ctrl-d to exit) ..."
  ur10 = armManipulation()   ## moveGroup  
  main()
