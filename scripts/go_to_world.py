#!/usr/bin/env python
 
import rospy
import numpy as np
from math import pi
import tf
import actionlib
from geometry_msgs.msg import Twist
from tf.listener import TransformListener
from geometry_msgs.msg import PoseStamped #/visp_auto_tracker/object_position
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String 
from std_msgs.msg import Int8

waypoints = [[[(-4.2623, 3.0751, 0.0),(0.0,0.0,0.0,1.0)],[(-3.08, 1.95, 0.0),(0.0, 0.0,0.0,1.0)]],
            [[(-6.2234, 3.0438, 0.0),(0.0,0.0,0.0,1.0)],[(-3.08,0.0,0.0),(0.0, 0.0,0.0,1.0)]],
            [[(-3.0263, -0.3530, 0.0),(0.0,0.0,0.0,1.0)],[(0.1,3.5,0.0),(0.0, 0.0,0.0,1.0)]],
            [[(-3.4866, -2.8865, 0.0),(0.0,0.0,0.0,1.0)],[(2.67,3.23,0.0),(0.0, 0.0,0.0,1.0)]],
            [[(-5.0346, -3.0536, 0.0),(0.0,0.0,0.0,1.0)],[(3.03,1.15,0.0),(0.0, 0.0,0.0,1.0)]]]



pose_camera_optical=np.zeros((4,1))
QR_message=[0.0,0.0,0.0,0.0,0,'ros']


def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
 
    return goal_pose
def go_to_world(pose_world,pose): 
    global client
    # add to the pose an offset
    observe_offset_scale_x = 0.25
    observe_offset_scale_y = 0.8
    new_x =   pose_world[0][0] - observe_offset_scale_x * np.sign(pose_world[0][0]-pose[0][0])
    new_y = pose_world[0][1] - observe_offset_scale_y * np.sign(pose_world[0][1]-pose[0][1])
    pose_world2 = [(new_x, new_y, pose_world[0][2]),  (pose_world[1][0], pose_world[1][1], pose_world[1][2], pose_world[1][3])]  
    goal = goal_pose(pose_world2)
    # go to observe point
    client.send_goal(goal)
    client.wait_for_result()
    rospy.sleep(5)

#def rotate():
   # global cmd_vel_pub, stop_wandering
    #fov_camera = 0.74839718
    #rotations = int(2*pi / fov_camera + 0.5)
    #print(rotations)
    #twist = Twist()
    #for i in range(rotations):
           #if(): #change here - if I see a qr code then I have to stop
            #stop_wandering = True
            #break;
        #twist.angular.z = fov_camera
        #cmd_vel_pub.publish(twist)
        #rospy.sleep(1)
        #wist.angular.z = 0.0
        #cmd_vel_pub.publish(twist)
        #rospy.sleep(1)

def rotate():  ## rotate to find and read QR codes
    
    rotations = int(angle_max/angle_goal)

    for i in range(rotations):
    #if flag_num_qrCode==2:
        #break
        print('rotate and looking for QR')
        # start observe
        move_cmd = Twist()
        move_cmd.angular.z = angle_speed
        for t in range(ticks):
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()
            
        move_cmd = Twist()
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(3)
        try:
                (trans, rot) = listener.lookupTransform('/odom', '/camera_optical_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


def qr_status_callback(msg):
    
    if(msg.data == 3):
        print('Qr code observed')
        rospy.sleep(5)
        #break

def QR_position_callback(msg):
    pose_camera_optical[0] = msg.pose.position.x
    pose_camera_optical[1] = msg.pose.position.y
    pose_camera_optical[2] = msg.pose.position.z
    pose_camera_optical[3] = 1.0

def QR_message_callback(msg):
    if len(msg.data.split('\r\n'))>5:
        QR_message[0] = float(msg.data.split('\r\n')[0].split('=')[1])
        QR_message[1] = float(msg.data.split('\r\n')[1].split('=')[1])
        QR_message[2] = float(msg.data.split('\r\n')[2].split('=')[1])
        QR_message[3] = float(msg.data.split('\r\n')[3].split('=')[1])
        QR_message[4] = int(msg.data.split('\r\n')[4].split('=')[1])
        QR_message[5] = msg.data.split('\r\n')[5].split('=')[1]




if __name__ == '__main__':
    rospy.init_node('go_to_world')
    listener = TransformListener()
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    qr_status = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_status_callback)
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, QR_position_callback, queue_size=10)
    rospy.Subscriber('/visp_auto_tracker/code_message', String, QR_message_callback, queue_size=10)
 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    
    rate = rospy.Rate(10.0)
    angle_speed = 1.0
    angle_goal= pi/6
    angle_max=2*pi
    angular_duration = angle_goal / angle_speed
    ticks = int(angular_duration * 10.0)
    #stop_wandering = False
    #while not stop_wandering:
    # do stuff
        #rotate()

    # go to observe point

    
    go_to_world([(-5.0346, -3.0536, 0.0),(0.0,0.0,0.0,1.0)],[(3.03,1.15,0.0),(0.0, 0.0,0.0,1.0)])
    rotate()
    

 


    
