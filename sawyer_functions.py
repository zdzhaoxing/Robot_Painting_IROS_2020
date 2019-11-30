from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
import copy
import numpy as np
import os
import sys
sys.path.insert(0, "/home/yayun/robot_painting")

import time

# Move to a place via absolute position
def move_to_position(self,pose_to_move):
	# retrieve current pose from endpoint
#	current_pose = self.endpoint_pose()
	ik_pose = Pose()
	ik_pose.position.x = copy.deepcopy(pose_to_move[0])
	ik_pose.position.y = copy.deepcopy(pose_to_move[1])
	ik_pose.position.z = copy.deepcopy(pose_to_move[2])
	ik_pose.orientation.x = copy.deepcopy(pose_to_move[3])
	ik_pose.orientation.y = copy.deepcopy(pose_to_move[4])
	ik_pose.orientation.z = copy.deepcopy(pose_to_move[5])
	ik_pose.orientation.w = copy.deepcopy(pose_to_move[6])
	joint_angles = self.ik_request(ik_pose)
	self.move_to_joint_positions(joint_angles)

def move_to_RelativP(self,pose_to_move):
	# retrieve current pose from endpoint
	current_pose = copy.deepcopy(self.endpoint_pose())
	ik_pose = Pose()
	ik_pose.position.x = copy.deepcopy(current_pose['position'].x + pose_to_move[0])
	ik_pose.position.y = copy.deepcopy(current_pose['position'].y + pose_to_move[1])
	ik_pose.position.z = copy.deepcopy(current_pose['position'].z + pose_to_move[2])
	ik_pose.orientation.x = copy.deepcopy(current_pose['orientation'].x + pose_to_move[3])
	ik_pose.orientation.y = copy.deepcopy(current_pose['orientation'].y + pose_to_move[4])
	ik_pose.orientation.z = copy.deepcopy(current_pose['orientation'].z + pose_to_move[5])
	ik_pose.orientation.w = copy.deepcopy(current_pose['orientation'].w + pose_to_move[6])
	joint_angles = self.ik_request(ik_pose)
	self.move_to_joint_positions(joint_angles)


def toArray(pos):
	converted = []
	converted.append(pos['position'].x)
	converted.append(pos['position'].y)
	converted.append(pos['position'].z)
	converted.append(pos['orientation'].x)
	converted.append(pos['orientation'].y)
	converted.append(pos['orientation'].z)
	converted.append(pos['orientation'].w)
	return converted


def move_by_step(limb, inipos, finpos, nsteps):
	# inipos, finpos are lists
	u = (np.array(finpos) - np.array(inipos)).tolist()

	for i in range(nsteps):
		cu = [float(i) / 100 * x for x in u]
		move_to_position(limb, [sum(x) for x in zip(inipos, cu)])

def move_up_by_normal(limb, Q, pos, N, d):
    '''
    This function move the roller to the position above the staring point of stroke
    pos is the starting point of stroke
    Q is the quaternion
    N is the normal of cooresponding surface
    d is the distance to move up
    '''
    x_d = d * N[0]
    y_d = d * N[1]
    z_d = d * N[2]
    x_n = pos[0] + x_d
    y_n = pos[1] + y_d
    z_n = pos[2] + z_d
    new_pose = [x_n, y_n, z_n] + Q
    move_to_position(limb, new_pose)
    
def move_up_current(limb, N, d):
    '''
    This function move up the roller
    N is the normal of cooresponding surface
    d is the distance to move up
    '''
    x_d = d * N[0]
    y_d = d * N[1]
    z_d = d * N[2]
    move_to_RelativP(limb, [x_d, y_d, z_d, 0, 0, 0, 0])

def divide_strokes(stroke):
    strokes = []
    for i in range(len(stroke)/2):
        stroke_1 = stroke[i*2]
        stroke_2 = stroke[i*2+1]
        strokes.append([stroke_1, stroke_2])

    return strokes

def dip(limb):
	# list with 3 position + 4 quaternion coordinates
    bucket_pos = [0.25015917249914493, 0.4346473734113633, 0.09622518966074645, 0.7106490800512902, 0.7024907908398066, -0.026623891545220848, 0.027852149046473623]
    # list of two lists with 3 position + 4 quaternion coordinates

    scratch_stroke = [ \
		[0.5956101383354518,0.4526094010821385, 0.14283991826219816, 0.7234978694349432, 0.6902700081139389, 0.00881403574438846, -0.0006794081085436936],
		[0.5368230171482965, 0.3030875981106746, 0.09478021824261358,0.6822005572920399, 0.730095469792962, 0.01300347247276418, 0.03733516200413731]]
    # just to get the current orientation
    # cur_pos = toArray(limb.endpoint_pose())	# it actually doesn't make sense to keep the same orientation
    # move to 10 cm above the bucket
    move_to_position(limb, bucket_pos[:2] + [bucket_pos[2] + 0.3] + bucket_pos[3:])
    time.sleep(0.1)
    # dip into bucket
    move_to_position(limb, bucket_pos + bucket_pos[3:])
    time.sleep(1.5)
    # move back to 10 cm above the bucket
    move_to_position(limb, bucket_pos[:2] + [bucket_pos[2] + 0.3] + bucket_pos[3:])
    time.sleep(10.0)
    # condition the roller
    for _ in range(3):
         move_by_step(limb, scratch_stroke[0], scratch_stroke[1], 100)
         move_to_position(limb, bucket_pos[:2] + [bucket_pos[2] + 0.2] + bucket_pos[3:])
     
    move_to_position(limb, bucket_pos[:2] + [bucket_pos[2] + 0.3] + bucket_pos[3:])
