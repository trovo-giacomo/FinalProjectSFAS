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
word = "-----"

global rotation
global translation

qr_messages = []
pos_worlds = []

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
    global cmd_vel_pub, state
    fov_camera = 0.523 # 30 deg
    twist = Twist()
        
    twist.angular.z = fov_camera
    cmd_vel_pub.publish(twist)
    # wait 1 sec
    state = 3


def stop():
    global cmd_vel_pub, state
    twist = Twist()
    #print(twist)
    #twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    #rospy.sleep(3)
    state = 4

 
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
    global msg_qr
    msg_qr = msg.data

def qr_tf_cov_callback(msg):
    global ob_postion_relative
    ob_postion_relative = msg.pose
    

def parse_message():
    global msg_qr, word, qr_messages
    # Read message
    content = str(msg_qr) #.copy()
    
    values = []
    for iter in range(5):
        idx_start = content.find('=')+1
        idx_end = content.find('\n')-1
        value = content[idx_start:idx_end]
        content = content[idx_end+2:]
        values.append(value)

    value = content[2:]
    values.append(value)
        
    id = int(values[4])
    qr_msg = {}
    qr_msg["pos_c"] = [float(values[0]), float(values[1])]
    qr_msg["pos_n"] = [float(values[2]), float(values[3])]
    qr_msg["id"] = id
    qr_msg["L"] = values[5]

    return qr_msg


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
    #print("a: "+str(a))
    #print("b: "+str(b))
    lenA = np.linalg.norm(a)
    lenB = np.linalg.norm(b)
    lenC = np.linalg.norm(a-b)
    #print(lenA)
    #print(lenB)
    #print(lenC)
    #rotAngle(np.dot(a,b)/(len)
    #temp = (math.pow(lenA,2)+math.pow(lenB,2) - math.pow(lenC,2))/2*lenA*lenB
    #print(temp)
    rotAngle= math.acos((math.pow(lenA,2)+math.pow(lenB,2) - math.pow(lenC,2))/(2*lenA*lenB))
    return rotAngle


if __name__ == '__main__':

    rospy.init_node('explore_waypoints')
    rate = rospy.Rate(1)
    ob_postion_relative = PoseWithCovarianceStamped()
    listener = tf.TransformListener()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    qr_status = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_status_callback)
    #qr_obj_pos = rospy.Subscriber('visp_auto_tracker/object_position', PoseStamped, qr_tf_callback)
    qr_obj_pos_cov = rospy.Subscriber('visp_auto_tracker/object_position_covariance', PoseWithCovarianceStamped, qr_tf_cov_callback)
    qr_msg = rospy.Subscriber('visp_auto_tracker/code_message', String, qr_msg_callback)
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    state = 1
    idx_waypoints = 0
    nr_qr_found = 0
    rotations = 12
    idx_rotations = 0
    msg_qr = ""
    previousMsg = {}
    ob_postion_relative = PoseWithCovarianceStamped
    num_period_steady = 0
    
    while not rospy.is_shutdown():
        if(state==1):
            print(" 1 - Go to waypoint "+str(idx_waypoints))
            pose = waypoints[idx_waypoints]
            goal = goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result()
            idx_waypoints = (idx_waypoints +1) % len(waypoints) # circular increment
            state+=1
        elif(state == 2):
            print(" 2 - rotate: "+str(idx_rotations)+"/"+str(rotations))
            if(idx_rotations < rotations):
                print("Rotate 30 degree")
                num_period_steady = 0
                rotate() # rotate 30 degree
                idx_rotations+=1
            else:
                print("No more rotations needed go to next waypoint")
                state=1 # no enough qr found go to next waypoint
        elif(state == 3):
                # stop and loo for qr code
                print(" 3 - Stop")
                stop()
        elif(state == 4):
            print(" 4 - look for QR code period "+str(num_period_steady)+"/3")
            if(num_period_steady > 3):
                print(" Continue to rotate")
                state = 2 # continue to rotate
            # look for qr code
            obj_msg = {}
            if(status_qr == 3):
                obj_msg = parse_message()
            print("prev: "+str(previousMsg) + "current: "+str(obj_msg))
            if(status_qr == 3 and previousMsg != obj_msg and num_period_steady >1):
                print("Got new QR code - get position in the world")
                # qr code detected and it is different from the previous one
                # get position realtive prosition from 'ob_postion_relative'
                pose_camera = [ob_postion_relative.pose.position.x, ob_postion_relative.pose.position.y, ob_postion_relative.pose.position.z, 1];
                previousMsg = obj_msg
                try:
                    # look up for the transformation between camera and odometry
                    (trans,rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
                    # apply the transformation
                    t = translation_matrix(trans)
                    matrix_rot = quaternion_matrix(rot)
                    T_odomo_camera = concatenate_matrices(t,matrix_rot)
                    p_qr = T_odomo_camera.dot(pose_camera)
                    #print(p_qr)
                    #print(msg_qr)
                    # update number of qr code seen
                    nr_qr_found+=1
                    state = 5  # go to check if I need to proceed on or stop
                    obj_msg["pos_w"] = [p_qr[0], p_qr[1]] 
                    pos_worlds.append([p_qr[0], p_qr[1]])
                    print(pos_worlds)

                    # append input message to the list of seen qr codes
                    qr_messages.append(obj_msg)
                    # Assign letter to position of id
                    word = word[:obj_msg["id"]-1] + obj_msg["L"] + word[obj_msg["id"]:]

                    # Count number of missing letters (-)
                    #missing_values = int(word.count('-'))
                    print("QR code met until now")
                    print(qr_messages)
                    print("word:"+word)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    print("Error in the listener lookup transformations")
            num_period_steady += 1
        elif(state == 5):
            print(" 5 - Check if 2 qr code found: "+str(nr_qr_found)+"/2")
            if(nr_qr_found == 2):
                print("Find hidden frame")
                state = 11
            else:
                print("Continue to rotate")
                state = 2
        elif(state == 10):
            print(" 10 - Stopping")
            stop()
            break
        elif(state==11):
            print("11 - Get hidden frame from the first two qr codes")
            #transInit(realPos1, realPos2, hiddenPos1, hiddenPos2)
            posW1 = np.array(qr_messages[0]["pos_w"])
            posW2 = np.array(qr_messages[1]["pos_w"])
            posH1 = np.array(qr_messages[0]["pos_c"])
            posH2 = np.array(qr_messages[1]["pos_c"])
            transInit(posW1, posW2, posH1, posH2)
            state=12
        elif(state == 12):
            print("12 - transfrom from hidden 2 world")
            next_pose = hidden2real(qr_messages[-1]["pos_n"])
            print("Next pose in the world")
            print(next_pose)
            state = 10
        #print("Nex loop")
        rate.sleep()
    print("Program terminated")

#go_to_word([(1.5, 2.4, 0.0),(0.0, 0.0, 0.0, 0.0)],[(-2.0, -0.5, 0.0),(0.0, 0.0, 0.0, 0.0)])
#go_to_word([],[])