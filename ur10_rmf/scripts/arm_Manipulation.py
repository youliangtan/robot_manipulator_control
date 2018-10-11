#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# check if current state reaches goal state, check all joints is within tolerance
def all_close(goal, actual, tolerance=0.001):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      print "Compare tolerance> ", abs(actual[index] - goal[index]), tolerance
      if abs(actual[index] - goal[index]) > tolerance: # checking here
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


def changeInOrientation( wpose_quaternion, delta_roll, delta_pitch, delta_yaw):

    qua = [ wpose_quaternion.x,
            wpose_quaternion.y, 
            wpose_quaternion.z, 
            wpose_quaternion.w]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(qua)
    quaternion = tf.transformations.quaternion_from_euler(roll + delta_roll , pitch + delta_pitch, yaw + delta_yaw)

    wpose_quaternion.x = quaternion[0]
    wpose_quaternion.y = quaternion[1]
    wpose_quaternion.z = quaternion[2]
    wpose_quaternion.w = quaternion[3]

    return wpose_quaternion


# control arm velocity of cartesian by adding travel time betwwen each way points
def controlArmVelocity(plan, numberOfWayPoints = 1, factor=1): 

  print "Number of Planned Waypoints ", numberOfWayPoints
  print "Time change factor as ", factor

  for i in range(1, numberOfWayPoints):

    time = plan.joint_trajectory.points[i].time_from_start.to_sec()
    plan.joint_trajectory.points[i].time_from_start = rospy.Time.from_sec( time*factor ) 
    plan.joint_trajectory.points[i].velocities = [ j/factor  for j in plan.joint_trajectory.points[i].velocities ]  #divide vel with factor
    plan.joint_trajectory.points[i].accelerations = [ j/factor  for j in plan.joint_trajectory.points[i].accelerations ]  #divide acc with factor

  return plan



## CLASS

class armManipulation(object):

  ## ------------------------------------- Start init -------------------------------------

  def __init__(self):
    super(armManipulation, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    # rospy.init_node('moveGroup_ur10_rmf', anonymous=True)
    
    moveit_commander.roscpp_initialize(sys.argv)

    ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
    ## the robot:
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot: (will pub collision obj )
    scene = moveit_commander.PlanningSceneInterface()

    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to one group of joints.  In this case the group is the joints in the Panda
    ## arm so we set ``group_name = panda_arm``. If you are using a different robot,
    ## you should change this value to the name of your robot arm planning group.
    ## This interface can be used to plan and execute motions on the Panda:
    group_name = "manipulator"
    group = moveit_commander.MoveGroupCommander(group_name)

    ## We create a `DisplayTrajectory`_ publisher which is used later to publish
    ## trajectories for RViz to visualize:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)




    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.group = group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


    ## Add Robot Base to scene
    base_name = "base_box"
    is_known = base_name in scene.get_known_object_names()
    print "is base added befor ???? ", is_known

    if (is_known == True): # check if base model is added before
      scene.remove_world_object(base_name)
      scene.remove_world_object("box")
      rospy.sleep(0.5)

    ## Create a box in the planning scene at the location of the left finger:
    box = geometry_msgs.msg.PoseStamped()
    box.header.frame_id = "base_link"
    box.pose.orientation.w = 1.0
    box.pose.position.z = -0.05
    print scene.add_box(base_name, box, size=(0.7, 0.7, 0.1))
    print "added base_box yo~"

    self.scene = scene
    rospy.sleep(1.5)  # crude method to ensure scene is loaded




  ## ------------------------------------- End init -------------------------------------

  #### Joints move to Joint States
  def go_to_joint_state(self, joint_goal):

    state = self.robot.get_current_state()
    self.group.set_start_state(state)
    group = self.group

    # The go command can be called with joint values, poses, or without any
    # group.go(joint_goal, wait=True)
    plan = group.plan (joints = joint_goal)

    plan = controlArmVelocity(plan, numberOfWayPoints=len(plan.joint_trajectory.points), factor=2  )

    group.execute(plan, wait=True)
    group.stop()

    current_joints = self.group.get_current_joint_values()

    return all_close(joint_goal, current_joints, tolerance=0.03)


  #### Eef move to Pose Goal
  def go_to_pose_goal(self):

    group = self.group

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    ## Planner to compute the plan and execute it.
    plan = group.go(wait=True)

    # plan = controlArmVelocity(plan, numberOfWayPoints=1, timeInterval=0.8  )

    group.stop()
    group.clear_pose_targets()

    current_pose = self.group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)



  ## ================================ Cartesian =================================


  ## Picking the beverage
  #### Eef Paths in Cartesian space
  def plan_cartesian_picking_path(self, scale=1):
    
    state = self.robot.get_current_state()
    self.group.set_start_state(state)
    group = self.group
    
    waypoints = []
    wpose = group.get_current_pose().pose


    ## first move
    wpose.position.z += scale * 0.1  # First move down (z)    
    waypoints.append(copy.deepcopy(wpose))

    ## second move
    wpose.orientation = changeInOrientation( wpose.orientation, 0, 0, 0.785) #45 degrees
    wpose.position.x -= scale * 0.1  # Second move forward/backwards in (x)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    ## third move
    wpose.position.x += scale * 0.06  # Second move forward/backwards in (x)
    wpose.position.y += scale * 0.06  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))


    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.2,         # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:

    plan = controlArmVelocity(plan, numberOfWayPoints=len(plan.joint_trajectory.points), factor=2  )

    return plan, fraction


  #Placing the beverage
  def plan_cartesian_placing_path(self, scale=1):

    state = self.robot.get_current_state()
    self.group.set_start_state(state)
    group = self.group
    
    waypoints = []
    wpose = group.get_current_pose().pose

    ## First move
    wpose.position.x -= scale * 0.06  # Second move forward/backwards in (x)
    wpose.position.y -= scale * 0.06  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    ## second move
    wpose.orientation = changeInOrientation( wpose.orientation, 0, 0, -0.785) #45 degrees
    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    wpose.position.y -= scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    ## Third move
    wpose.position.z -= scale * 0.1  # First move down (z)    
    waypoints.append(copy.deepcopy(wpose))

    # Same as picking
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.2, 0.0)

    plan = controlArmVelocity(plan, numberOfWayPoints=len(plan.joint_trajectory.points), factor=2  )

    return plan, fraction    



  # Placing the beverage in payload
  def plan_cartesian_payload_path(self, scale=1):

    state = self.robot.get_current_state()
    self.group.set_start_state(state)
    group = self.group
    
    waypoints = []
    wpose = group.get_current_pose().pose
    wpose.position.x += scale * 0.06  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    # Same as picking
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.2, 0.0)
    
    plan = controlArmVelocity(plan, numberOfWayPoints=len(plan.joint_trajectory.points), factor=2 )

    return plan, fraction    




  #### Displaying a Planned Trajectory
  def display_trajectory(self, plan):

    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)


  #### Executing a Plan
  def execute_plan(self, plan):
    state = self.robot.get_current_state()
    self.group.set_start_state(state)
    self.group.execute(plan, wait=True)


  #### Ensuring Collision Object state Updates Are Receieved
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    box_name = self.box_name
    scene = self.scene

    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False


  #### Adding Objects to the Planning Scene
  def add_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    ## Create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "ee_link"
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  #### Attaching Objects to the Robot
  def attach_box(self, timeout=4):

    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## Next, attach the box to the eef. Manipulating objects requires the robot be able to touch them without the planning scene
    ## reporting the contact as a collision. By adding link names to the ``touch_links`` array, planning scene will ignore 
    ## collisions between those links and the box. grasping group depends on model of robot manipulator
    grasping_group = 'manipulator'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)

    # Wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  #### Detaching Objects from the Robot
  def detach_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link
    
    scene.remove_attached_object(eef_link, name=box_name)
    
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  ## Removing Objects from the Planning Scene  
  def remove_box(self, timeout=4):

    box_name = self.box_name
    scene = self.scene

    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)