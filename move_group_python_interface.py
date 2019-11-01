#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import intera_interface
from moveit_commander import MoveGroupCommander, roscpp_initialize

import pyrealsense2 as rs
import time
import numpy as np
import random
import math
import open3d as o3d
import pcl
import pptk
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from point_cloud_functions import *
from sawyer_functions import *

sys.path.insert(0, "/home/yayun/robot_painting")
# rospy.init_node('move_group_python_interface',
#             anonymous=True)
# limb = intera_interface.Limb('right')

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

class MoveGroupPythonInteface(object):
    """MoveGroupPythonInteface"""
    def __init__(self):
        super(MoveGroupPythonInteface, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        # joint_state_topic = ['joint_states:=/robot/joint_states']
        moveit_commander.roscpp_initialize(sys.argv)
        # moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move_group_python_interface',
                    anonymous=True)
        self.limb = intera_interface.Limb('right')
        # roscpp_initialize(sys.argv)

        # self.limb.move_to_neutral()
        ## Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        ## the robot:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        ## to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to one group of joints.  
        group_name = "right_arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        # group.set_planner_id("RRTkConfigDefault")
        group.set_planner_id("RRTConnectkConfigDefault")
        ## We create a `DisplayTrajectory`_ publisher which is used later to publish
        ## trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        # Misc variables
        self.box_name = 'roller'
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## Planning to a Joint Goal
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        # joint_goal = group.get_current_joint_values()
        joint_goal = [[]for i in range(7)]
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable

        # current_joints = self.group.get_current_joint_values()
        # return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = position[3]
        pose_goal.orientation.y = position[4]
        pose_goal.orientation.z = position[5]
        pose_goal.orientation.w = position[6]
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
    
        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        # current_pose = self.group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)

    def plan_pose_goal(self, position):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = position[3]
        pose_goal.orientation.y = position[4]
        pose_goal.orientation.z = position[5]
        pose_goal.orientation.w = position[6]
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]

        group.set_pose_target(pose_goal)
        ## Set velocity
        group.set_max_velocity_scaling_factor(0.1)
        ## Now, we call the planner to compute the plan and execute it.
        plan = group.plan()
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        # group.clear_pose_targets()
        return plan

    def plan_cartesian_path(self, initpose, endpose, quaternion, step=5,scale=0.1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group
        midpoint = geometry_msgs.msg.Pose()
        # set the fitst point
        midpoint.position.x = initpose[0]
        midpoint.position.y = initpose[1]
        midpoint.position.z = initpose[2]
        midpoint.orientation.x = quaternion[0]
        midpoint.orientation.y = quaternion[1]
        midpoint.orientation.z = quaternion[2]
        midpoint.orientation.w = quaternion[3]

        waypoints = []
        waypoints.append(midpoint)
        dx = (endpose[0] - initpose[0])*1.0 / step
        dy = (endpose[1] - initpose[1])*1.0 / step
        dz = (endpose[2] - initpose[2])*1.0 / step

        for i in range(step):
            midpoint.position.x = initpose[0]+(i+1)*dx
            midpoint.position.y = initpose[1]+(i+1)*dy
            midpoint.position.z = initpose[2]+(i+1)*dz
            waypoints.append(midpoint)
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ################################################################################
        # waypoints = []

        # wpose = group.get_current_pose().pose
        # wpose.position.z -= scale * 0.1  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        #################################################################################
        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        group.set_max_velocity_scaling_factor(0.1)

        (plan, fraction) = group.compute_cartesian_path(
                                                waypoints,   # waypoints to follow
                                                0.01,        # eef_step
                                                0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        # print(plan)
        return plan, fraction

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
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

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan

        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Receieved
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear.
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


    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene at the location of the left finger:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "right_gripper_tip"
        box_pose.pose.orientation.w = 1.0
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = 'right_arm'
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

    def add_obstacle(self, obstacle, timeout=4):
        """ Add one obstacle to the 
        """
        scene = self.scene
        obstacle_pose = geometry_msgs.msg.PoseStamped()
        obstacle_pose.header.frame_id = "world"
        obstacle_name = obstacle[0]
        obstacle_pose.pose.position.x = obstacle[1]
        obstacle_pose.pose.position.y = obstacle[2]
        obstacle_pose.pose.position.z = obstacle[3]
        obstacle_pose.pose.orientation.x = obstacle[4]
        obstacle_pose.pose.orientation.y = obstacle[5]
        obstacle_pose.pose.orientation.z = obstacle[6]
        obstacle_pose.pose.orientation.w = obstacle[7]
        obstacle_size = obstacle[8]
        scene.add_box(obstacle_name, obstacle_pose, obstacle_size)
        
    def remove_obstacle(self, obstacle_name):
        scene = self.scene
        scene.remove_world_object(obstacle_name)

def obstacles_on_face(face_id,T_matrix,quaternion,paint_points,width=0.10,offset=0.05):
    """Given paint_points on a face, output the obstacles' names, poses in a list
    """
    obstacles = []
    for i in range(len(paint_points)/2):
        stroke_1 = paint_points[i*2]
        stroke_2 = paint_points[i*2+1]
        x_o = (stroke_1[0] + stroke_2[0]) / 2.0 + width / 2.0
        y_o = (stroke_1[1] + stroke_2[1]) / 2.0 
        z_o = offset / 2.0
        center = trans_to_coordinates(T_matrix, [[x_o, y_o, z_o]])
        obstacle_name = "face %d obstacle %d" % (face_id, i)
        obstacles.append([obstacle_name, center[0][0], center[0][1], center[0][2], quaternion[0], quaternion[1], quaternion[2],quaternion[3],[width, stroke_1[1] - stroke_2[1],offset]])
    # print obstacles
    return obstacles

def paint_proceeing(self, paint_points, T_matrix, coordinate_planes, norm):
    """ The procedure to paint the whole object.
        Including six parts:
        1. Add obstacles
        2. Dip in the paint box
        3. Move to the place above the beginning point and face it
        4. Remove all the obstacles
        5. paint stroke by stroke
        6. move up a liitle bit when finish the last stroke
        Input: points to paint on each faces, transfer matrix, coordinates of each planes
        Output: the plan of the whole painting process
    """
   
    num_planes = len(paint_points)
    
    # compute the quaternions of planes and end-effector
    quaternion_plane = []
    for i in range(num_planes):
        quaternion_plane.append(coor_to_quater(coordinate_planes[i]))

    quaternion_end = []
    for i in range(num_planes):
        quaternion_end.append(coor_to_quater(norm_to_base(coordinate_planes[i])))

    # compute the obstacles on all the planes
    obstacles = []
    for i in range(num_planes):
        obstacles.append(obstacles_on_face(i, T_matrix[i], quaternion_plane[i], paint_points[i]))

    # compute the paint strokes on each plane
    paint_points_end = []
    for i in range(num_planes):
        paint_points_end.append(trans_to_end(paint_points[i]))
    
    paint_points_joint = []
    for i in range(num_planes):
        paint_points_joint.append(trans_to_joint(paint_points[i]))
    
    paint_points_world = []
    for i in range(num_planes):
        stroke = trans_to_coordinates(T_matrix[i], paint_points_end[i])
        paint_points_world.append(stroke)

    paint_points_world_joint = []
    for i in range(num_planes):
        stroke = trans_to_coordinates(T_matrix[i], paint_points_joint[i])
        paint_points_world_joint.append(stroke)

    for i in range(num_planes):
        # 1. add obstacles on all planes
        adding_obstacles(self, num_planes, obstacles)
        rospy.sleep(1)
        # 2. dip in the paint box
        # dip(self)
        
        # 3. move to the place above the beginning point
        beginning_point = move_to_begin(self, quaternion_end[i], norm[i], paint_points_world[i][0])
        rospy.sleep(1)
        
        print ("begin point: ", beginning_point)
        # 4. remove all the obstacles 
        rm_obstacles(self, num_planes, obstacles)

        # 5. paint stroke by stroke
        paint_strokes_plane(self, beginning_point, quaternion_end[i], norm[i], paint_points_world_joint[i], obstacles, num_planes)

        # 6. move up when finish painting
        num_stroke_points = len(paint_points_world[i])
        move_up_last(self, quaternion_end[i], norm[i], paint_points_world_joint[i][num_stroke_points-1])
        

def dip(self, bucket_pose, scratch_1, scrathc_2):
    """ Move the end-effector to a paint box and dip it
    """
    

def paint_line(self, inipos, endpos, nsteps=20):
    """ Paint stroke in straight line
    """
    u = (np.array(endpos) - np.array(inipos)).tolist()
    way_points = []
    for i in range(nsteps):
        cu = [float(i) / (nsteps-1) * x for x in u]
        target_pos = [sum(x) for x in zip(inipos, cu)]
        self.go_to_pose_goal(target_pos)
    #     way_points.append(target_pos)
    # return way_points
    
def move_to_begin(self, quaternion, norm, beginning_point, offset=0.02):
    """ move to the beginning point of one face
        with an offseti
    """
    beginning_pose = copy.deepcopy(beginning_point)
    beginning_pose[0] += norm[0]*offset
    beginning_pose[1] += norm[1]*offset
    beginning_pose[2] += norm[2]*offset
    plan = self.plan_pose_goal(beginning_pose + quaternion)
    self.execute_plan(plan)
    rospy.sleep(1)
    cur_pose = []
    end_point_pose = copy.deepcopy(self.limb.endpoint_pose())
    cur_pose.append(end_point_pose['position'].x)
    cur_pose.append(end_point_pose['position'].y)
    cur_pose.append(end_point_pose['position'].z)
    return cur_pose

def paint_strokes_plane(self, beginning_point, quaternion, norm, paint_points, obstacles, num_planes):
    """ paint the strokes on one plane
    """
    
    plane_strokes = divide_strokes(paint_points)
    n = len(plane_strokes)
    for i in range(n-1):
        self.limb.set_joint_position_speed(0.05)
        move_to_firstPose(self, beginning_point, quaternion, plane_strokes[i])
        paint_one_stroke(self, quaternion, plane_strokes[i])
        beginning_point = move_to_another_stroke(self, quaternion, norm, plane_strokes[i],plane_strokes[i+1], obstacles, num_planes)
    # paint the last stroke
    self.limb.set_joint_position_speed(0.05)
    move_to_firstPose(self, beginning_point, quaternion, plane_strokes[n-1])
    beginning_point = paint_one_stroke(self, quaternion, plane_strokes[n-1])
   
def move_to_firstPose(self, beginning_point, quaternion, plane_strokes):
    """Move to the first point of one stroke
    """
    firstPose = beginning_point + quaternion
    secondPose = plane_strokes[0] + quaternion

    self.limb.set_joint_position_speed(0.05)
    move_by_step(self.limb, firstPose, secondPose, 30)

def divide_strokes(stroke):
    """ Divide strokes into two pieces
    """
    strokes = []
    for i in range(len(stroke)/2):
        stroke_1 = stroke[i*2]
        stroke_2 = stroke[i*2+1]
        strokes.append([stroke_1, stroke_2])
    return strokes

def paint_one_stroke(self, quaternion, paint_points,nstep=30):
    """ paint one stroke from up to down
    """
    ############################################
    # self.go_to_pose_goal(paint_points[0]+quaternion)
    # rospy.sleep(1)
    # self.go_to_pose_goal(paint_points[1]+quaternion)
    # rospy.sleep(1)
    ############################################
    # inipos = paint_points[0]+quaternion
    # endpos = paint_points[1]+quaternion
    # paint_line(self, inipos, endpos)
    ############################################
    firstPose = paint_points[0]+quaternion
    secondPose = paint_points[1]+quaternion
    self.limb.set_joint_position_speed(0.1)
    move_by_step(self.limb, firstPose, secondPose, nstep)
    
    
def move_to_another_stroke(self, quaternion, norm, stroke_now, stroke_next, obstacles, num_planes, offset=0.02):
    """ move the roller up a liitle after one stroke and move to another stroke
    """
    ############################################
    # position_up = stroke_now[1]
    # position_next = stroke_next[0]
    # position_up[0] += norm[0] * offset
    # position_up[1] += norm[1] * offset
    # position_up[2] += norm[2] * offset
    # self.go_to_pose_goal(position_up+ quaternion)
    # adding_obstacles(self, num_planes, obstacles)
    # rospy.sleep(1)
    # position_next[0] += norm[0] * offset
    # position_next[1] += norm[1] * offset
    # position_next[2] += norm[2] * offset
    # self.go_to_pose_goal(position_next+quaternion)
    # rm_obstacles(self, num_planes, obstacles)
    # rospy.sleep(1)
    #############################################
    # Move up a liitle bit
    position_now = stroke_now[1]
    position_up = copy.deepcopy(position_now)
    position_up[0] += norm[0]*offset
    position_up[1] += norm[1]*offset
    position_up[2] += norm[2]*offset
    self.limb.set_joint_position_speed(0.05)
    move_by_step(self.limb,position_now + quaternion,position_up + quaternion,30)
    # Move to the next beginning point
    position_next = copy.deepcopy(stroke_next[0])
    position_next[0] += norm[0]*offset
    position_next[1] += norm[1]*offset
    position_next[2] += norm[2]*offset
    self.limb.set_joint_position_speed(0.05)
    move_by_step(self.limb,position_up+quaternion,position_next+quaternion,30)
    return position_next

def move_up_last(self, quaternion, norm, last_point, offset=0.02):
    """ When finishing a plane, move the roller up away from the plane
    """
    #############################################
    # last_point[0] += norm[0] * offset
    # last_point[1] += norm[1] * offset
    # last_point[2] += norm[2] * offset 
    # self.go_to_pose_goal(last_point+quaternion)
    #############################################
    firstPose = last_point
    secondPose = copy.deepcopy(firstPose)
    secondPose[0] += norm[0]*offset
    secondPose[1] += norm[1]*offset
    secondPose[2] += norm[2]*offset
    self.limb.set_joint_position_speed(0.05)
    move_by_step(self.limb,firstPose+quaternion,secondPose+quaternion,30)

def adding_obstacles(self, num_planes, obstacles):
    """ Add all the obstacles at once
    """
    for i in range(num_planes):
        for j in range(len(obstacles[i])):
            self.add_obstacle(obstacles[i][j])
            # rospy.sleep(1)

def rm_obstacles(self, num_planes, obstacles):
    """ Remove all the obstacles at once
    """
    for i in range(num_planes):
        for j in range(len(obstacles[i])):
            self.remove_obstacle(obstacles[i][j][0])
            # rospy.sleep(1)

def main():
    try:
        print "============ Press `Enter` to begin by setting up the moveit_commander (press ctrl-d to exit) ..."
        raw_input()
        sawyer = MoveGroupPythonInteface()

        # print "============ Press `Enter` to execute a movement using a joint state goal ..."
        # raw_input()
        # sawyer.go_to_joint_state()
        
        # print "============ Press `Enter` to print current state ..."
        # raw_input()    
        # print (sawyer.group.get_current_pose())
        # print (sawyer.group.get_current_joint_values())

        # print "============ Press `Enter` to plan a movement using a pose goal ..."
        # raw_input()
        # initial_pos = [0.4509654575135258, 0.15888974540100548, 0, 0.7083853421810006, 0.7058196679997667, 0.002524309902387455, 0.0015592008605980927]
        # plan_pose = sawyer.plan_pose_goal(initial_pos)
        # print(plan_pose)

        # print "============ Press `Enter` to execute a movement using a pose goal ..."
        # raw_input()
        # # from O to A
        # quternion_A = [-0.4370368119199106, 0.8990329878065346, -0.011881114529464509, -0.024440764719494492]
        # position_A = [0.7650203416295934, -0.038778085364988966, 0.08421510648669164]
        # pose_A = position_A + quternion_A

        # sawyer.go_to_pose_goal(pose_A)

        # position_B = [0.8333584405128465, 0.0482218654761585, 0.0781955463085853]
        # pose_B = position_B + quternion_A

        # sawyer.go_to_pose_goal(pose_B)


        # print "============ Press `Enter` to plan and display a Cartesian path ..."
        # raw_input()
        # initpose = [0.8137436981113302, -0.18559849979205845, 0.6]
        # endpose = [0.8203607426993076, -0.1706561963408544, 0.4]
        # quaternion = [0.6343047681502666,
        #                 0.41282272750535726,
        #                 -0.5478264854735972,
        #                 0.3565403182958706]
        # plan, fraction = sawyer.plan_cartesian_path(initpose,endpose,quaternion)
        # sawyer.execute_plan(plan)
        # print "============ Press `Enter` to display a saved trajectory ..."
        # raw_input()
        # sawyer.display_trajectory(plan_pose)

        # print "============ Press `Enter` to execute a saved path ..."
        # raw_input()
        # sawyer.execute_plan(plan_pose)

        # print "============ Press `Enter` to add a box to the planning scene ..."
        # raw_input()
        # sawyer.add_box()

        # print "============ Press `Enter` to attach a Box to the Sawyer robot ..."
        # raw_input()
        # sawyer.attach_box()

        # print "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # raw_input()
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # sawyer.execute_plan(cartesian_plan)

        # print "============ Press `Enter` to detach the box from the Sawyer robot ..."
        # raw_input()
        # sawyer.detach_box()

        # print "============ Press `Enter` to remove the box from the planning scene ..."
        # raw_input()
        # sawyer.remove_box()

        # print "============ Python demo complete!"

        # print "============ Press `Enter` to add obstacles to the scene ..."
        # raw_input()
        # paint_points = [[-0.152164075827403, 0.06312112430324546, 0.0],
        #                [-0.152164075827403, -0.06756694477166009, 0.0],
        #             [-0.062164075827402986, 0.059665826333581085, 0.0],
        #             [-0.062164075827402986, -0.06768274325186335, 0.0],
        #               [0.027835924172597018, 0.05777574695889559, 0.0],
        #              [0.027835924172597018, -0.06744416612868576, 0.0],
        #               [0.030658510531015953, 0.05777574695889559, 0.0],
        #              [0.030658510531015953, -0.06744416612868576, 0.0]]

        # quaternion = [0.07970304342911529, 0.7154099335950291, 0.689875966102968, 0.07685833185275998]

        # T_matrix = [[-0.9754804433858951, 0.007995266133919505, 0.21994085634803096, 0.6771035569255117], 
        # [0.22008612989386087, 0.03543715252326577, 0.9748365522739622, -0.04941303931376871],
        # [8.673617379884035e-19, 0.9993399241201617, -0.036327896436615215, 0.35256899479078335],
        # [0, 0, 0, 1]]

        # obstacles = obstacles_on_face(0, T_matrix, quaternion, paint_points)

        # for i in range(len(obstacles)):
        #     sawyer.add_obstacle(obstacles[i])  
 
        # print "============ Press `Enter` to add rollor to the end effector"
        
        # sawyer.add_box()
        
        print "============ Press `Enter` to exercute the whole painting process"
        raw_input()
        paint_points_ps = [[[-0.11458770296129049, 0.05751002308459166, 0.0],
                            [-0.11458770296129049, -0.05995467554471054, 0.0],
                            [-0.024587702961290488, 0.05487568517665639, 0.0],
                            [-0.024587702961290488, -0.06531071246491246, 0.0],
                            [0.026513973820830844, 0.05487568517665639, 0.0],
                            [0.026513973820830844, -0.05992877083102324, 0.0]],
                            [[-0.11542831560835119, 0.04398986512131664, 0.0],
                            [-0.11542831560835119, -0.053819911262338094, 0.0],
                            [-0.02542831560835119, 0.05615137300161682, 0.0],
                            [-0.02542831560835119, -0.05635441238163097, 0.0],
                            [0.009607852173321899, 0.06067992949363693, 0.0],
                            [0.009607852173321899, -0.05635441238163097, 0.0]],
                            [[-0.06153950857753609, 0.04331333707071566, 0.0],
                            [-0.06153950857753609, -0.058398422910562076, 0.0],
                            [-0.03789300355547335, 0.04331333707071566, 0.0],
                            [-0.03789300355547335, -0.058398422910562076, 0.0]]]
                            

        T_matrix =  [[[-0.9164056553602904,
                        -0.053977561011439724,
                        0.3965943743057043,
                        0.7459419133212066],
                        [0.400250764925786,
                        -0.12358587792483114,
                        0.9080340610097576,
                        -0.1791046423426851],
                        [-6.938893903907228e-18,
                        0.9908647504502343,
                        0.13485935753663836,
                        0.28366686000327734],
                        [0, 0, 0, 1]],
                        [[-0.9125784908665999,
                        -0.40655827986762033,
                        0.04371341989274601,
                        0.7070511717774695],
                        [0.4089015749635096,
                        -0.9073487709701858,
                        0.0975587505621619,
                        -0.22953612876995425],
                        [-1.3877787807814457e-17,
                        0.10690450360005856,
                        0.9942692930539644,
                        0.3421064482380621],
                        [0, 0, 0, 1]],
                        [[-0.7005338509706431,
                        0.18562492441969494,
                        -0.6890542149050559,
                        0.6338963858335308],
                        [-0.713619172699445,
                        -0.18222120160808933,
                        0.6764193300315001,
                        -0.1688943249053543],
                        [-2.7755575615628914e-17,
                        0.965576936923561,
                        0.26011762508779207,
                        0.2954644491025264],
                        [0, 0, 0, 1]]]

        coordinate_planes = [[[-0.9164056553602904, 0.400250764925786, -6.938893903907228e-18],
                                [-0.053977561011439724, -0.12358587792483114, 0.9908647504502343],
                                [0.3965943743057043, 0.9080340610097576, 0.13485935753663836]],
                                [[-0.9125784908665999, 0.4089015749635096, -1.3877787807814457e-17],
                                [-0.40655827986762033, -0.9073487709701858, 0.10690450360005856],
                                [0.04371341989274601, 0.0975587505621619, 0.9942692930539644]],
                                [[-0.7005338509706431, -0.713619172699445, -2.7755575615628914e-17],
                                [0.18562492441969494, -0.18222120160808933, 0.965576936923561],
                                [-0.6890542149050559, 0.6764193300315001, 0.26011762508779207]]]

        n_planes = [[0.3965943743057043, 0.9080340610097576, 0.13485935753663836],
                    [0.04371341989274601, 0.0975587505621619, 0.9942692930539644],
                    [-0.6890542149050559, 0.6764193300315001, 0.26011762508779207]]
        
        table_pose = ['table', 0.6, -0.05, -0.4, 1, 0, 0, 0, [0.02,0.02,0.6]]
        sawyer.add_obstacle(table_pose)
        paint_proceeing(sawyer, paint_points_ps, T_matrix, coordinate_planes, n_planes)
        
        # print "============ Press `Enter` to add obsctacles to the scene"
        # raw_input()
        
        # # print(paint_plan)
    
        # num_planes = 3

        # quaternion_plane = []
        # for i in range(num_planes):
        #     quaternion_plane.append(coor_to_quater(coordinate_planes[i]))

        # obstacles = []
        # for i in range(num_planes):
        #     obstacles.append(obstacles_on_face(i, T_matrix[i], quaternion_plane[i], paint_points_ps[i]))
        
        # adding_obstacles(sawyer, num_planes, obstacles)

        print "============ The movement ends"

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()
