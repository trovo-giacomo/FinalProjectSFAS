#!/usr/bin/env python

import rospy
import actionlib
import numpy as np
import math
global rotation
global translation


def transInit(realPos1, realPos2, hiddenPos1, hiddenPos2): #give args as numpy arrays
	realPosVector=realPos1-realPos2
	hiddenPosVector=hiddenPos1- hiddenPos2
	theta=getRot(realPosVector,hiddenPosVector) #gets angle in radians
	global rotation
	c, s = np.cos(theta), np.sin(theta)
	rotation = np.array(((c, -s), (s, c)))
	global translation
	translation = realPos1- rotation.dot(hiddenPos1)

def hidden2real(hidden):
	global rotation
	global translation
	real = rotation.dot(hidden)+translation
	return real


def getRot(a,b):
	lenA = np.linalg.norm(a)
	lenB = np.linalg.norm(b)
	lenC = np.linalg.norm(a-b)
	rotAngle= math.acos((math.pow(lenA,2)+math.pow(lenB,2) - math.pow(lenC,2))/2*lenA*lenB)
	return rotAngle

real1 = np.array([1,1])
real2 = np.array([2,1])
hid1 = np.array([1,2])
hid2 = np.array([1,1])

transInit(real1, real2, hid1, hid2)
print hidden2real(np.array([1,0]))