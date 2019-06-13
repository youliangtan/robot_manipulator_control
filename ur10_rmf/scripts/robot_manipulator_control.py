#!/usr/bin/env python

"""
=========================================================
Creator: Tan You Liang
Date: Sept 2018
Description:  Arm control process
              Edit `motion_config`.yaml for motion control
Gripper:      Open: 0, close: 1
==========================================================
"""


import sys
import os 
import copy
import rospy
import yaml
import signal
import datetime
from time import sleep
from termcolor import colored

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list
import geometry_msgs.msg
import tf
from arm_manipulation import ArmManipulation

import numpy as np
from math import pi

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray # temp solution

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose

# from dynamixel_gripper.msg  import grip_state
from rm_msgs.msg import grip_state
from rm_msgs.msg import ManipulatorState


MOTION_CONFIG_PATH = "../config/motion_config.yaml"
ROS_TIME_RATE = 6 #hz


class RobotManipulatorControl():
  def __init__(self):

    rospy.init_node('robot_manipulator_control_node', anonymous=True)
    rospy.Subscriber("/gripper/state", grip_state, self.gripperState_callback)
    rospy.Subscriber("/ur10/target_pose_2d", Pose2D, self.targetPose_2d_callback)
    rospy.Subscriber("/ur10/target_pose_3d", Pose, self.targetPose_3d_callback)
    rospy.Subscriber("/ur10/reset", grip_state, self.reset_callback)
    self.gripper_pub = rospy.Publisher('/gripper/command', Int32, queue_size=10)
    self.ur10 = ArmManipulation()   ## moveGroup  
    self.init_state_config()


  # Read Yaml file
  def load_motion_config(self, path):
    dir_path = os.path.dirname(os.path.realpath(__file__))
    full_path = dir_path + "/" + path
    with open(full_path, 'r') as stream:
      try:
        self.yaml_obj = yaml.load(stream)
      except yaml.YAMLError as exc:
        print("Error in loading Yaml" + exc)
        exit(0)

    self.enable_gripper =  self.yaml_obj['enable_gripper'] # bool
    self.log_motion_error = self.yaml_obj['log_motion_error']

    #create log file, same dir as config.yaml
    if (self.log_motion_error == True):
      self.log_file = open("error_log.txt", "w+")


  # for initialization
  def init_state_config(self):
    self.gripper_state = -1
    self.arm_motion_state = ''
    self.rate = rospy.Rate( ROS_TIME_RATE ) # 6hz
    self.enable_gripper = False
    self.yaml_obj = []
    self.new_motion_request = False 
    self.motion_request = 'Nan' 
    self.motion_group_progress = 1.0
    self.is_success = True
    self.target_pose_2d =  np.array([0,0,0])   # target pose detected by 2d pose estimation topic respect to sensor pose
    self.target_pose_3d =  np.array([0,0,0])


  # ***************************************************************************************************************
  ##############################################  ROS CallBack Zone   #############################################
  #################################################################################################################


  # Reset Handler
  def reset_callback(self, data):
    if ( data.data == True):
      print( colored(" \n =================== \t RESET IS TRIGGERED \t =================== \n", "white", "on_magenta"))
      self.reset()
      self.load_motion_config(path=MOTION_CONFIG_PATH)
      self.init_state_config()
    else:
        print(" -- Reset is 'False': Nothing Happened --")


  def gripperState_callback(self, data): 
    # rospy.loginfo( "I heard gripper state is {}".format(data.gripper_state))
    self.gripper_state = data.gripper_state


  def motionService_callback(self, data): 
    self.motion_request = data.data 
    self.new_motion_request = True 

  # 2d pose estimation callback
  def targetPose_2d_callback(self, data): 
    # print(" @@@@ Pose callback:" , data)
    self.target_pose_2d =  np.array([data.x, data.y, data.theta])

  # 3d position callback (x,y,z)
  def targetPose_3d_callback(self, data): 
    # print(" @@@@ Pose callback:" , data)
    self.target_pose_3d =  np.array([data.position.x, data.position.y, data.position.z])


  # Timer to pub Manipulator` State in every interval
  # *Note: when motion is executing, ros callback is being blocked
  def timer_pub_callback(self, event):

      eef_pose = self.ur10.get_eef_pose()
      arm_joints = self.ur10.get_arm_joints()

      print ('[CallBack] pub timer called at: ' + str(event.current_real))
      qua = [ eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(qua)
      print ("Eef_pose: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]" %(eef_pose.position.x, eef_pose.position.y, eef_pose.position.z, roll, pitch, yaw))
      print ("Arm Joints: {%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}" %(arm_joints[0], arm_joints[1], arm_joints[2], arm_joints[3], arm_joints[4], arm_joints[5]))

      # Send rm_msgs, TODO
      msg = ManipulatorState()
      msg.gripper_state = self.gripper_state
      msg.arm_motion_state = self.motion_request
      msg.arm_motion_progress = self.motion_group_progress   # float, show fraction of completion
      msg.x = eef_pose.position.x
      msg.y = eef_pose.position.y
      msg.z = eef_pose.position.z
      msg.roll = roll
      msg.pitch = pitch
      msg.yaw = yaw
      self.RMC_pub.publish(msg)

      # TODO: temp solution to pub to ros1_ros2_bridge
      if (self.is_success == True):
        error_flag = 0.0
      else:
        error_flag = 1.0

      msg = Float32MultiArray()
      motion_group_num = self.motion_request[1:] # convert: e.g. 'G23' to 23
      if (motion_group_num.isdigit()):
        motion_group_num = float(motion_group_num)
      else:
        motion_group_num = -1.0
      msg.data =  [ self.gripper_state, 
                    motion_group_num, 
                    self.motion_group_progress, 
                    error_flag,  # new
                    eef_pose.position.x,
                    eef_pose.position.y,
                    eef_pose.position.z,
                    roll,
                    pitch,
                    yaw ]
      self.rm_bridge_pub.publish(msg)


  # TODO: external moveit stop to stop all trajectory motion
  def stop_motion_callback(self):
    #sub to some topic and when theres's call back, stop motion
    pass



  # *******************************************************************************************************************
  ##############################################   Private Class Function   ###########################################
  #####################################################################################################################


  # Open gripper, TODO: return success or fail
  def open_gripper(self): 

    if (self.enable_gripper == True):
      gripper_command = 0       # command to open gripper
      rospy.loginfo(gripper_command) #printout
      self.gripper_pub.publish(gripper_command)

      # wait until get state is open
      while (self.gripper_state == 1):
        if (self.gripper_state == -1):
          print("Error From Gripper!!")
          return False  
        print("gripper still close... with state {}".format(self.gripper_state))
        self.rate.sleep()
    
    return True


  # Close Gripper, TODO: return success or fail
  def close_gripper(self):

    if (self.enable_gripper == True):
      gripper_command = 1       #command to close gripper
      rospy.loginfo(gripper_command)
      self.gripper_pub.publish(gripper_command)

      # wait until get state is close
      while (self.gripper_state == 0):
        if (self.gripper_state == -1):
          print("Error From Gripper!!")
          return False  
        print("gripper still open... with state {} ".format(self.gripper_state))
        self.rate.sleep()

    return True


  # Manage cartesian motion sequence from .yaml 
  # motion_coeff: higher level coeff from motion group sequence
  def manage_cartesian_motion_list(self, cartesian_motion, motion_coeff):
    motion_list = []

    for cartesian_id in cartesian_motion:
      # Support cooficient handling
      cartesian_coeff, filtered_cartesian_id = self.get_coeff_from_id(ch='C', id=cartesian_id)
      cartesian_data = self.yaml_obj['cartesian_motion'][filtered_cartesian_id]
      cartesian_data = map(lambda x: x*cartesian_coeff*motion_coeff, cartesian_data)
      motion_list.append(cartesian_data)

    return motion_list


  # get coefficient from a motion id or cartesian id
  def get_coeff_from_id(self, ch, id):
    char_idx = id.find(ch)
    filtered_id = id[char_idx:len(id)]
    coefficient = id[0:char_idx]

    if (len(coefficient)> 0 ): # check if theres cooficient infront of Cartesian motion
      if (coefficient == '-'):
        coefficient = -1
      else:
        coefficient = float(coefficient)
    else:
      coefficient = 1
    return (coefficient, filtered_id)


  # get 2d pose adjustment according to detected target obj
  # now is being constraint in 2D space (TODO)
  def get_pose_adjustment(self, expected_target_pose, target_pose_tolerance):
      
      # TODO: replace expected pos with get_eef_pose, add diff to target 
      current_target_pose = self.target_pose_2d +  np.array(self.yaml_obj['fix_laser_pose']) # tf respect to baselink
      # TODO: handle Yaw angle
      current_target_pose[2] =  current_target_pose[2] - pi
      print("@@YAW: ", current_target_pose[2])
      pose_adjustment = [0,0,0]

      for i in range(3):
        # TODO: use transformation equation
        diff =  current_target_pose[i] - expected_target_pose[i] 
        if ( abs(diff) < target_pose_tolerance[i]):
          pose_adjustment[i] = diff
        else:
          if (diff > 0):
            pose_adjustment[i] = target_pose_tolerance[i]
          else:
            pose_adjustment[i] = -target_pose_tolerance[i]

      print( colored(" Dynamic Pose Adjustment [x, y, theta]: {} unit".format(pose_adjustment), 'green'))
      pose_3d = [pose_adjustment[0], pose_adjustment[1], 0, 0, 0, pose_adjustment[2]] # 2d to 3d xyzrpy

      return pose_3d 


  ##################################################################################################################
  ############################################## Public class function #############################################


  """ 
  Execute single 'motion' according to 'motion_config.yaml' 
  @Input: String, motion_id 
  @Return: Bool, Success? 
  """
  def execute_motion(self, motion_id, coeff=1):
    try:
      motion_descriptor = self.yaml_obj['motion'][motion_id]
      motion_type = motion_descriptor['type']
      is_success = False
      planned_fraction = None
      print( colored(" -- Motion: {}, {} ".format(motion_id, motion_descriptor), 'blue') )

      ## **Joint Motion
      if ( motion_type == 'joint_goal'):
        joint_goal = motion_descriptor['data']
        motion_time_factor = motion_descriptor['timeFactor']
        is_success = self.ur10.go_to_joint_state(joint_goal, motion_time_factor)
        
      ## **Pose Goal Motion
      elif ( motion_type == 'pose_goal'):
        pose_goal = motion_descriptor['data']
        motion_time_factor = motion_descriptor['timeFactor']
        is_success = self.ur10.go_to_pose_goal(pose_goal, motion_time_factor)

      ## **Cartesian Motion
      elif ( motion_type == 'cartesian'):
        motion_time_factor = motion_descriptor['timeFactor']
        motion_sequence = motion_descriptor['sequence']
        cartesian_motion_list = self.manage_cartesian_motion_list( motion_sequence, coeff )
        cartesian_plan, planned_fraction = self.ur10.plan_cartesian_path(cartesian_motion_list, motion_time_factor)
        print(" -- Planned fraction: {} ".format(planned_fraction))
        if (planned_fraction == 1.0):
          is_success = self.ur10.execute_plan(cartesian_plan)

      ## **Pose Goal Motion, 2D Pose Estimation Result: TODO
      elif ( motion_type == '2d_dynamic_cartesian' ):
        # check if skipping dynamic planning
        if (self.yaml_obj['skip_dynamic_cartesian'] == True):
          is_success = True
          print(" -- Skip Cartesian RePositioning ")
        else:
          motion_time_factor = motion_descriptor['timeFactor']
          # get 2d pose adjustment according to detected target obj
          if ( self.target_pose_2d.tolist() != [0,0,0]): 
            cartesian_motion = self.get_pose_adjustment( motion_descriptor['target'], motion_descriptor['tolerance'] )
            cartesian_plan, planned_fraction = self.ur10.plan_cartesian_path( [cartesian_motion], motion_time_factor)
            print(" -- Dynamic Cartesian Planned fraction: {} ".format(planned_fraction))
            if (planned_fraction == 1.0):
              is_success = self.ur10.execute_plan(cartesian_plan)
          else:
            is_success = False
            print(" -- No Target, Skip Cartesian RePositioning, end task! ")

      ## **Pose Goal Motion, 3D Pose Estimation Result: TODO
      elif ( motion_type == '3d_dynamic_cartesian' ):
        pass
        #developing!!!
        cartesian_plan, planned_fraction = self.ur10.plan_cartesian_path( [cartesian_motion], motion_time_factor, motion_type="absolute")


      ## **Close Gripper Motion
      elif ( motion_type == 'eef_grip_obj'):
        self.ur10.add_box()
        self.ur10.attach_box()
        is_success = self.close_gripper()

      ## **Open Gripper Motion
      elif ( motion_type == 'eef_release_obj'):
        self.ur10.detach_box()
        self.ur10.remove_box()
        is_success = self.open_gripper()
                
      else:
        print(colored("Error!! Invalid motion type in motion descriptor, motion_config.yaml", 'red', 'on_white'))
        exit(0)

      print(colored(" -- Motion success outcome: {}".format(is_success), 'green'))
      
      # Log error msg if there's failed outcome
      if (is_success == False and self.log_motion_error == True):
        self.log_file = open("error_log.txt", "a")
        self.log_file.write(" Motion Group: {}, gripper state: {} \n".format(self.motion_request, self.gripper_state))
        self.log_file.write(" -- DateTime: {} \n".format(datetime.datetime.now()))
        self.log_file.write(" -- Failed Motion: {} \n".format(motion_descriptor))
        self.log_file.write(" -- Cartessian plan fraction: {} \n".format( planned_fraction ))
        self.log_file.write(" -- current eef pose: {} \n".format(self.ur10.get_eef_pose()))
        self.log_file.write(" -- current arm joints: {} \n\n".format(self.ur10.get_arm_joints()))
        self.log_file.close() 


    except KeyError, e:
      print(colored("ERROR!!! invalid key in dict of .yaml, pls check your input related to motion_config.yaml",'red'))  
    except IndexError, e:
      print(colored("ERROR!!! invalid index in list of .yaml, pls check your input related to motion_config.yaml",'red'))  

    rospy.sleep(0.15) # make sure joint update is latest, maybe?

    return is_success



  """ 
  @input: String, Target Motion Group ID
  @return: Bool, success? 
  """
  def execute_motion_group(self, target_id):
    
    try:
      all_motion_groups = self.yaml_obj['motion_group']
      numOfMotionGroup = len(all_motion_groups)
      self.motion_group_progress = 0
      for i in range(numOfMotionGroup):
        
        if ( all_motion_groups[i]['id'] == target_id):  # if found id
          motion_sequences = all_motion_groups[i]['sequence']
          # Loop thru each 'motion'
          fraction = 1.0/len(motion_sequences)
          for motion_id in motion_sequences:
            # find -1 in motion sequences
            coeff, filtered_motion_id = self.get_coeff_from_id(ch='M', id=motion_id)
            is_success = self.execute_motion(filtered_motion_id, coeff=coeff)
            if (is_success == False):
              print(colored("Returned Is_success: False after execute a motion", 'red'))
              return False
            self.motion_group_progress = self.motion_group_progress + fraction
            self.rate.sleep()
          return True
      
    except KeyError, e:
      print(colored("ERROR!!! invalid key in dict of .yaml, pls check your input related to motion_config.yaml",'red'))  
    except IndexError, e:
      print(colored("ERROR!!! invalid index in list of .yaml, pls check your input related to motion_config.yaml",'red'))  
    
    return False
    


  """ 
  Execute series of motion groups, motions, and cartesian motions
  - Motion Groups are executed sequencially. Press enter to continue
  """
  def execute_all_motion_group(self):
    
    # Solely for printout
    eef_pose = self.ur10.get_eef_pose()
    arm_joints = self.ur10.get_arm_joints()
    qua = [ eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(qua)
    print ("Eef_pose: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]" %(eef_pose.position.x, eef_pose.position.y, eef_pose.position.z, roll, pitch, yaw))
    print ("Arm Joints: {%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}" %(arm_joints[0], arm_joints[1], arm_joints[2], arm_joints[3], arm_joints[4], arm_joints[5]))


    try:

      # Loop thru each 'motion group'
      for obj, i in zip(self.yaml_obj['motion_group'], range(99)):
        motion_sequences = obj['sequence']
        print( colored(" =================== Motion_Group {}: {} =================== ".format(i, motion_sequences), 'blue', attrs=['bold']) )
        print( colored(" -- Press `Enter` to execute a movement -- ", 'cyan') )
        raw_input()

        # Loop thru each 'motion'
        for motion_id in motion_sequences:
          # find coeff infront of motionID
          coeff, filtered_motion_id = self.get_coeff_from_id(ch='M', id=motion_id)
          self.is_success = self.execute_motion(filtered_motion_id, coeff)
        
        # for printout
        eef_pose = self.ur10.get_eef_pose()
        arm_joints = self.ur10.get_arm_joints()
        qua = [ eef_pose.orientation.x, eef_pose.orientation.y, eef_pose.orientation.z, eef_pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(qua)
        print ("Eef_pose: [%.2f,%.2f,%.2f,%.2f,%.2f,%.2f]" %(eef_pose.position.x, eef_pose.position.y, eef_pose.position.z, roll, pitch, yaw))
        print ("Arm Joints: {%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}" %(arm_joints[0], arm_joints[1], arm_joints[2], arm_joints[3], arm_joints[4], arm_joints[5]))

      print(colored(" =================== All Motion Completed!  ===================", 'green'))

    except rospy.ROSInterruptException:
      return
    except KeyboardInterrupt:
      print("ctrl-c, exit!!! :'(")
      exit(0)


  """
  Execute a service node to call a specific motion group
   @Sub: '/ur10/motion_group_id', input group num id
   @Pub: '/ur10/manipulator_state', current state of the manipulator
  """
  def execute_motion_group_service(self):
    
    try:
      rospy.Subscriber("/ur10/motion_group_id", String, self.motionService_callback)
      self.RMC_pub = rospy.Publisher("/ur10/manipulator_state", ManipulatorState, queue_size=10)
      self.rm_bridge_pub = rospy.Publisher("/ur10/rm_bridge_state", Float32MultiArray, queue_size=10) # Temp Solution for RMC Pub
      rospy.Timer(rospy.Duration(1.5), self.timer_pub_callback)
      print (colored(" ------ Running motion group service ------ ", 'green', attrs=['bold']))

      while(1):
        # check if new request by user
        if (self.new_motion_request == True):
          print (" [Service]:: New Motion Group Request!! : {} ".format(self.motion_request) )
          self.new_motion_request = False
          self.is_success = self.execute_motion_group( self.motion_request )

        self.rate.sleep()

    except KeyboardInterrupt:
      print("ctrl-c, exit!!! :'(")
      exit(0)

              

# ***************************************************************************************************************
#################################################################################################################
#################################################################################################################



def signal_handler(sig, frame):
  print('You pressed Ctrl+C!, end program...')
  sys.exit(0)



if __name__ == '__main__':
  print(colored("  -------- Begin Python Moveit Script --------  " , 'white', 'on_green'))
  signal.signal(signal.SIGINT, signal_handler)
  robot_manipulator_control = RobotManipulatorControl()
  robot_manipulator_control.load_motion_config( path=MOTION_CONFIG_PATH )

  # robot_manipulator_control.execute_all_motion_group()
  robot_manipulator_control.execute_motion_group_service()
  # robot_manipulator_control.execute_motion_group("G5")
  
