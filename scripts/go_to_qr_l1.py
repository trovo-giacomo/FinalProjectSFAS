#!/usr/bin/env python
import numpy as np
import rospy
import actionlib
import math
import geometry_msgs
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
import tf
import tf_conversions
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix, concatenate_matrices, translation_matrix
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, String

 
#[(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
#[(0.13, 1.93, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]

waypoints = [  
    [(-4.2623, 3.0751, 0.0), (0.0, 0.0,  math.pi/2)],   #4
    [(-6.2234, 3.0438, 0.0), (0.0, 0.0,  math.pi/2)], #5 
    [(-3.0263, -0.3530, 0.0), (0.0, 0.0, 0.0)], #3
    [(-3.4866, -2.8865, 0.0), (0.0, 0.0,  0.0)],       #2 
    [(-5.0346, -3.0536, 0.0), (0.0, 0.0, -math.pi/2)], #1
]


 
def goal_pose(pose):  
    print(pose)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])) 
    
    return goal_pose


if __name__ == '__main__':
    steady = False
    rospy.init_node('go_to_qr')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()

    while True:
        for pose in waypoints:   
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            rospy.sleep(3)