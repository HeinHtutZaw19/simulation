#!/usr/bin/env python
import rospy
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import dynamic_reconfigure.client


def all_close(goal, actual, tolerance):
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
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)

    
    robot = moveit_commander.RobotCommander()

    
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)


   
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    
    move_group = self.move_group

    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0
    joint_goal[7] = 0

    
    move_group.go(joint_goal, wait=True)

    move_group.stop()


    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self,x,y,z,roll,pitch,yaw):
    
    move_group = self.move_group

    
    pose_goal = geometry_msgs.msg.Pose()
    temp = tf.transformations.quaternion_from_euler(roll* pi/180, pitch* pi/180, yaw* pi/180)

    pose_goal.position.x = x/100
    pose_goal.position.y = y/100
    pose_goal.position.z = z/100
    pose_goal.orientation.x = temp[0]
    pose_goal.orientation.y = temp[1]
    pose_goal.orientation.z = temp[2]
    pose_goal.orientation.w = temp[3]
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    
    move_group = self.move_group

    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction



  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);


  def execute_plan(self, plan):
    move_group = self.move_group

    move_group.execute(plan, wait=True)


tutorial = MoveGroupPythonIntefaceTutorial()
def callback(config):
    #global tutorial
    if config.send==True:
        rospy.loginfo("Config set to {x}, {y}, {z}, {roll}, {pitch}, {yaw}, {send}".format(**config))
        tutorial.go_to_pose_goal(config.x,config.y,config.z,config.roll,config.pitch,config.yaw)
    
if __name__ == "__main__":
    client = dynamic_reconfigure.client.Client("server", timeout=30, config_callback=callback)
    
    time=0
    while not rospy.is_shutdown():
      time+=1
    