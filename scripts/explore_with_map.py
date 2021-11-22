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
    [(-4.370,  0.500, 0.0), (0.0, 0.0,  math.pi)],   #1
    [(-4.542,  2.000, 0.0), (0.0, 0.0,  math.pi/2)], #2 
    [(-3.821, -2.000, 0.0), (0.0, 0.0, -math.pi/2)], #3
    [( 0.956,  0.158, 0.0), (0.0, 0.0,  0.0)],       #4 
    [( 1.068, -2.022, 0.0), (0.0, 0.0, -math.pi/2)], #5
    [( 4.066,  1.116, 0.0), (0.0, 0.0,  math.pi/2)], #6
    [( 5.702, -0.178, 0.0), (0.0, 0.0,  0.0)]        #7
]

def rotate():
    global cmd_vel_pub, stop_wandering, steady
    fov_camera = 0.74839718
    rotations = int(2*math.pi / fov_camera + 0.5)
    #print(rotations)
    twist = Twist()
    for i in range(rotations):
        
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        steady = True
        rospy.sleep(3)

        if(num_qrs >= num_max_qrs):
            stop_wandering = True
            break;

        twist.angular.z = fov_camera
        steady = False
        cmd_vel_pub.publish(twist)
        rospy.sleep(1)

    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    steady = True
    rospy.sleep(3)
    steady = False
        

 
def goal_pose(pose):  
    print(pose)
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])) 
    
    return goal_pose

def qr_status_callback(msg):
  global status_qr
  #print(msg)
  status_qr = msg.data
  
def qr_msg_callback(msg):
    global lastValidQRMessage, status_qr, p_qr, stop_wandering, num_seen_valid_qr, num_max_valid_qr, num_qrs, steady, ob_postion_relative
    if(status_qr == 3 and steady):
        #print(msg.data)
        num_seen_valid_qr+=1
        #print(p_qr)
        print("seen "+str(num_seen_valid_qr)+" number of time")
        tr = ob_postion_relative.covariance[0] + ob_postion_relative.covariance[7] #36 value of a 6x6 matrix
        print("Trace: "+str(tr))
        if(lastValidQRMessage != msg.data and msg.data != "" and tr < 1e-8):
            #I have a new QR code on the seen
            num_qrs+=1
            print(p_qr)
            print("Data: ")
            print(msg.data)
            print("Seen "+str(num_qrs)+" valid qr codes")
            print("num of seen QR codes: "+str(num_seen_valid_qr)+ ", max:"+str(num_max_valid_qr))
            lastValidQRMessage = msg.data
    else:
        num_seen_valid_qr = 0


def qr_tf_callback(msg):
    global ob_postion_relative, p_qr
    ob_postion_relative = msg.pose
    pose_camera = [ob_postion_relative.position.x, ob_postion_relative.position.y, ob_postion_relative.position.z, 1];
    try:
        (trans,rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
        
        t = translation_matrix(trans)
        matrix_rot = quaternion_matrix(rot)
        T_odomo_camera = concatenate_matrices(t,matrix_rot)
        p_qr = T_odomo_camera.dot(pose_camera)
        #print(p_qr)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error in the listener lookup transformations")

def qr_tf_cov_callback(msg):
    global ob_postion_relative, p_qr
    ob_postion_relative = msg.pose
    pose_camera = [ob_postion_relative.pose.position.x, ob_postion_relative.pose.position.y, ob_postion_relative.pose.position.z, 1];
    try:
        (trans,rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
        
        t = translation_matrix(trans)
        matrix_rot = quaternion_matrix(rot)
        T_odomo_camera = concatenate_matrices(t,matrix_rot)
        p_qr = T_odomo_camera.dot(pose_camera)
        #print(p_qr)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("Error in the listener lookup transformations")

if __name__ == '__main__':
    steady = False
    rospy.init_node('patrol')
    ob_postion_relative = PoseWithCovarianceStamped()
    listener = tf.TransformListener()
    p_qr = []
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    qr_status = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_status_callback)
    #qr_obj_pos = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, qr_tf_callback)
    qr_obj_pos_cov = rospy.Subscriber('visp_auto_tracker/object_position_covariance', PoseWithCovarianceStamped, qr_tf_cov_callback)
    qr_msg = rospy.Subscriber('visp_auto_tracker/code_message', String, qr_msg_callback)
    lastValidQRMessage = ""
    last_p_qr = []
    status_qr = 1
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    
    num_max_qrs = 2
    num_seen_valid_qr = 0
    stop_wandering = False
    num_qrs = 0
    num_max_valid_qr = 3
    while not stop_wandering:
        for pose in waypoints:   
            if(num_qrs >= num_max_qrs):
                stop_wandering = True
                print("Stop wandering: "+str(stop_wandering))
                break;
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            rotate()
            rospy.sleep(3)


#go_to_word([(1.5, 2.4, 0.0),(0.0, 0.0, 0.0, 0.0)],[(-2.0, -0.5, 0.0),(0.0, 0.0, 0.0, 0.0)])
#go_to_word([],[])