#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8, String
from final_project.msg import qr 

valid_state = False
word = "-----"

def code_message_callback(msg, qr_pub_code):
	global valid_state
	global word

	if valid_state == True:
		# Read message
		content = msg.data

		qr_msg = qr()
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
		qr_msg.x = float(values[0])
		qr_msg.y = float(values[1])
		qr_msg.x_next = float(values[2])
		qr_msg.y_next = float(values[3])
		qr_msg.id = id
		qr_msg.letter = values[5]

		# Check if word is long enough for id
		#diff = id - len(word)
		#if diff > 0:
		#	word = word + "x"*(diff)

		# Assign letter to position of id
		word = word[:id-1] + values[5] + word[id:]

		# Count number of missing letters (-)
		missing_values = int(word.count('-'))
		qr_msg.missing_letters = missing_values

		# Publish code and missing qr codes to read
		qr_pub_code.publish(qr_msg)

	else:
		pass
		# Publish default message?




def status_callback(msg):
	global valid_state

	if msg.data == 3: 	# found QR-code
		valid_state = True
	else:
		valid_state = False
		

	


rospy.init_node('qr_code')
qr_pub_code = rospy.Publisher('/msg_qr_code', qr, queue_size=1)

qr_status = rospy.Subscriber('/visp_auto_tracker/status', Int8, status_callback)
qr_code = rospy.Subscriber('/visp_auto_tracker/code_message', String, code_message_callback, qr_pub_code)

rospy.spin()