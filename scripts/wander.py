#!/usr/bin/env python
# BEGIN ALL
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
 

def qr_message_callback(msg):
  global stop_wandering, num_seen_valid_qr, num_max_valid_qr
  #print(msg)
  if(msg.data == 3):
    num_seen_valid_qr+=1
    print("seen "+str(num_seen_valid_qr)+" number of time")
    if(num_seen_valid_qr >= num_max_valid_qr):
      print("STOP")
      stop_wandering = True;
  else:
    num_seen_valid_qr = 0


def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  # look backwards
  #for i in range(len(msg.ranges)/2-12, len(msg.ranges)/2+12):
  #  tmp.append(msg.ranges[i])

  # look forward
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)
 
stop_wandering = False 
num_max_valid_qr = 3;
num_seen_valid_qr = 0;
g_range_ahead = 1 # anything to start
scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
#try:
qr_message = rospy.Subscriber('visp_auto_tracker/status', Int8, qr_message_callback)
#except e:
#  print("Please start \n roslaunch final_project qr_visp.launch\nif you want to stop when you find the QR code.")

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('wander')
state_change_time = rospy.Time.now() + rospy.Duration(1)
driving_forward = True
rate = rospy.Rate(60)
 
while (not rospy.is_shutdown()) and (not stop_wandering):
  #print g_range_ahead
  if g_range_ahead < 0.8:
    # TURN
    driving_forward = False
    #print "Turn"
   
  else: # we're not driving_forward
    driving_forward = True # we're done spinning, time to go forward!
    #DRIVE
    #print "Drive"
   
  twist = Twist()
  if driving_forward:
    twist.linear.x = 0.4
    twist.angular.z = 0.0
  else:
    twist.linear.x = 0.0
    twist.angular.z = 0.3
  cmd_vel_pub.publish(twist)
 
  rate.sleep()
# END ALL

#stop moving the robot
twist.linear.x = 0.0
twist.angular.z = 0.0
cmd_vel_pub.publish(twist)