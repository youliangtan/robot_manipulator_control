#!/usr/bin/env python
# license removed for brevity
# This script gets the current load in each servo and publishes the average load as a rostopic
# Created by Poh Yong Keat 2018

# TODO: change msg name convention: load_state -> LoadState

import rospy
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
from dynamixel_gripper.msg import load_state

# initialise variables
global load_left
global load_right
global load

load_left = 0
load_right = 0
load = 0

# Callback function
def callback(data):
    global load
    global load_left
    global load_right
    data = data.motor_states
    for motor in data:
        if(motor.id==1):
            load_left=motor.load
        if(motor.id==2):
            load_right=motor.load

    # Find average load
    load = (abs(load_left)+abs(load_right))/2

    rospy.loginfo("\nLeft load value: %f\nRight load value: %f\nAverage load value: %f\n-" % (load_left,load_right,load))

# main function
def getLoadValue():

    # Initialise variable(s)
    loadState = load_state()
    rospy.init_node('GripperLoadValue', anonymous=True)

    # Node subscribes to motor state and publishes load data
    pub = rospy.Publisher('/gripper/load', load_state, queue_size=10)
    rospy.Subscriber('/motor_states/pan_tilt_port', MotorStateList, callback)

    # Node refresh rate
    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        global load
        global load_left
        global load_right

        loadState.load_left = load_left
        loadState.load_right = load_right
        loadState.avg_load = load

        # publish load data to rostopic
        pub.publish(loadState)
        rate.sleep()


if __name__ == '__main__':
    try:
        getLoadValue()
    except rospy.ROSInterruptException:
        pass
