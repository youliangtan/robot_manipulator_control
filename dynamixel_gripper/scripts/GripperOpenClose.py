#!/usr/bin/env python
# license removed for brevity
# This scripts runs a node that opens and closes the dyanmixel gripper via ros topics
# Created by Poh Yong Keat 2018

import rospy
from std_msgs.msg import Float64, Int32
from dynamixel_msgs.msg import MotorStateList
from dynamixel_gripper.msg  import grip_state

# define constants
global LGrip_open_angle
global LGrip_close_angle
global RGrip_open_angle
global RGrip_close_angle

# RGrip_open_angle = 3.8009
# RGrip_close_angle = 2.7164
# LGrip_close_angle = 3.1717
# LGrip_open_angle = 2.2355

RGrip_open_angle = rospy.get_param("/RGrip_controller/motor/open_angle")
RGrip_close_angle = rospy.get_param("/RGrip_controller/motor/close_angle")
LGrip_close_angle = rospy.get_param("/LGrip_controller/motor/close_angle")
LGrip_open_angle = rospy.get_param("/LGrip_controller/motor/open_angle")

step_release_angle = 0.000174

# initialise variables
# gripper_state: 0 - open, 1 - close
global gripper_state
global feedback_load
global feedback_load_max
global present_angle
global user_command
global servo_temp

gripper_state = 0
feedback_load = 0
servo_temp = 0
feedback_load_max = 0.3
user_command = True
present_angle = [0,0]

# callback functions
def callback(data):
    global gripper_state
    global user_command
    gripper_state = int(data.data)
    user_command = True #indicate that a user command has been issued
    rospy.loginfo("Grip mode switched to %s" % gripper_state)

def callback_state(data):
    global feedback_load
    global present_angle
    global servo_temp
    feedback_load = 0
    servo_temp = 0
    data = data.motor_states
    if(len(data)==1): rospy.loginfo("Only 1 motor detected!")
    for i in range(len(data)):
        feedback_load = (feedback_load + abs(data[i].load))/(i+1)
        servo_temp = (servo_temp + data[i].temperature)/(i+1)
        # Map position from (0,1023) to (0,5.233), 5.233 is the maximum joint angle (300 deg)
        present_angle[i] = (data[i].position) * (5.2333/1023)

# main function
def open_and_close():

    global user_command

    # Initialise variable(s)
    msg = grip_state()
    rospy.init_node('GripperOpenClose', anonymous=True)

    # 3 publishers - 2 for controlling each servo, 1 for updating gripper status
    LGripPub = rospy.Publisher('/LGrip_controller/command', Float64, queue_size=10)
    RGripPub = rospy.Publisher('/RGrip_controller/command', Float64, queue_size=10)
    pub_state = rospy.Publisher('/gripper/state', grip_state, queue_size=10)

    # 2 subscribers - 1 to listen to open/close commands, 1 for getting current servo state
    rospy.Subscriber('/gripper/command', Int32, callback)
    rospy.Subscriber('/motor_states/pan_tilt_port', MotorStateList, callback_state)

    # node refresh rate 1Hz
    rate = rospy.Rate(1)

    # continuous loop
    while not rospy.is_shutdown():

        rospy.loginfo("Gripper Load: %f, Gripper State %d" % (feedback_load,gripper_state))

        # Differentiate between user-triggered change in gripper_state and internal gripper_state
        if(user_command == True):
            # open state
            if(gripper_state == 0):
                LGripPub.publish(LGrip_open_angle)
                RGripPub.publish(RGrip_open_angle)

            # closed state
            elif(gripper_state == 1):
                rospy.loginfo(feedback_load)
                LGripPub.publish(LGrip_close_angle)
                RGripPub.publish(RGrip_close_angle)

            else:
                rospy.loginfo("Invalid gripper state (%s) given. Please provide 0 or 1 only." % gripper_state)

            user_command = False

        # Inbuilt looping safety mechanism to prevent overloading i.e. check that a good grip is made and does not go over limit to prevent overload
        elif(abs(feedback_load)>=feedback_load_max):
            if(gripper_state == 1):
                # gradual but slow release of grip angles to maintain grip torque (using step release)
                new_left_angle = present_angle[0]-step_release_angle
                new_right_angle = present_angle[1]+step_release_angle
                LGripPub.publish(new_left_angle)
                RGripPub.publish(new_right_angle)
                rospy.loginfo("High load detected in closed pos, releasing joint angles.")

            elif(gripper_state == 0):
                # gradual but slow release of grip angles to maintain grip torque (using step release)
                new_left_angle = present_angle[0]+step_release_angle
                new_right_angle = present_angle[1]-step_release_angle
                LGripPub.publish(new_left_angle)
                RGripPub.publish(new_right_angle)
                rospy.loginfo("High load detected in open pos, releasing joint angles.")

        # publish latest gripper state data
        msg.gripper_state = gripper_state
        msg.left_pos = present_angle[0]
        msg.right_pos = present_angle[1]
        msg.avg_load = feedback_load
        msg.avg_temp = servo_temp
        pub_state.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        print "GripperOpenClose node has started."
        open_and_close()
    except rospy.ROSInterruptException:
        pass
