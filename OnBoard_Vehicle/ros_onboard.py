import serial
import datetime
import sys
from sys import platform

"""
In this test code we are testing basic vehicle control over the network
we use ROS middleware to send the control commands 
This script runs at the onboard vehicle end. 
Receives joystick messages by subscribing to topics, 
1. steering
2. break
3. throttle 
then writes the command to serial interface in the callback function
"""

def breakpoint():
	inp = input("Waiting for input")

if platform == "linux" or platform == "linux2":
	ser = serial.Serial('/dev/ttyUSB0')
elif platform == "darwin":
	pass
elif platform == "win32":
	# Windows...
	ser = serial.Serial('COM6')

import rospy
from std_msgs.msg import String

# msg_arr = [0]*9
# count_n = 0
def callback(target):

	# global count_n
	# if target.data == "N":
		# msg_arr.insert(0, 0)
		# msg_arr.pop()
		# count_n += 1
	# else:
		# count_n = 0
	
	

	# average = sum(msg_arr[2:6]) / 5

	if target.data == 'N':
		print("Emergency brake activated")
		target = "0T"
		ser.write((target).encode('utf-8'))
		target = "320B"
		ser.write((target).encode('utf-8'))
	else:
		print(target)
		ser.write((target.data).encode('utf-8'))
		


def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('break', String, callback)
	rospy.Subscriber('throttle', String, callback)
	rospy.Subscriber('steering', String, callback)
	rospy.Subscriber('HeartBeat', String, callback)

	rospy.spin()


if __name__ == '__main__':
	listener()
