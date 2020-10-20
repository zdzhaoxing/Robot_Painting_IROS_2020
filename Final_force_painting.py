#!/usr/bin/env python

'''
10/09/2019
Copyright Yayun Du  duyayun1hit@yahoo.com.hk
Copy and redistribution is not permitted. Written permission is required.
'''


import sys
import struct
import rospy
from std_msgs.msg import Float32
# from std_msgs.msg import Int16
import argparse
import copy
import math
import numpy as np
import rospkg
import os
import intera_interface

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

rospy.init_node('listener', anonymous=True)

limb = intera_interface.Limb('right')

def move_to_position(self,pose_to_move):
    # retrieve current pose from endpoint
#    current_pose = self.endpoint_pose()
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

def addNum(inn):
    inn = inn + 1
    return inn

def callback(data):
    #0.05 here is based on we have done the planning, known the total length of path and decided to move to the goal in 100 steps.
	move_to_RelativP(limb,[0.0,0.05,0,0.0,0.0,0.0,0.0])
	print(data.data)
    # now = rospy.Time.now()
    # seconds = rospy.get_time()
    # print(seconds)
    # rospy.sleep(1)
    # dis=13.75

    
    # # this is in centimeter, ALSO: Float32 is not float. data.data is float but data is Float32 type
    # 13.75 is distance between the ultrasonic sensor and the desk when the applied force is 8N
	if data.data > 13.75 and data.data < 16: # The roller deviate from the desk surface
		move_to_RelativP(limb,[0.0,0.05, -(data.data-13.75)/100.0,0.0,0.0,0.0,0.0])
	else if data.data >16: 
        n = addNum(n)
        if n == 5:
            print("Ooooops, the roller is near the edge of the object surface!!!")
            move_to_RelativP(limb,[0.0, 0.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068])#rotation along the z-axis in counter-clockwise
            n = 0
            if data.data > 16:
                move_to_RelativP(limb,[0.0, 0.0, 0.0, 0.0, 0.0, 0.7071068, 0.7071068])#rotation along the z-axis in counter-clockwise
                l = addNUm(l)
                if l == 1:
                    move_to_RelativP(limb,[0.0,-0.05, -(data.data-13.75)/100.0,0.0,0.0,0.0,0.0]) #now should move along -y direction cuz the arm orientation changes
            else:
                move_to_RelativP(limb,[-0.05,0.0, -(data.data-13.75)/100.0,0.0,0.0,0.0,0.0]) #now should move along -x direction cuz the arm orientation changes
        else if n > 5: # This means the larger number is just a noise not meaning the roller is near an edge, this is based on the inspection that the 
        # noise frequency of HC-SR04 is not freequent at all. so 5 will not slow the painting speed a lot and can avoid noise spikes
            move_to_RelativP(limb,[0.0,0.05, -(data.data-13.75)/100.0,0.0,0.0,0.0,0.0]) #initially want to use median filter but this turns out to be enough
            # Actually it will be better for the denoising happens in the C code of ultrasonic sensor
		
def distance():
	# move_to_position(limb,[0.4497198372937747, 0.16101522789929917, 0.21502854095979806, 0.7086823392857167, 0.7055238099751253, 0.00030457596684265055, -0.0023243872717985913])
	rospy.Subscriber("distance", Float32, callback)
	rospy.spin()
#loops
if __name__ == '__main__':
    n = 0
    l = 0
	distance()
	
