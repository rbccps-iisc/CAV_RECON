#!/usr/bin/env python3 

import rospy

import rospy
from std_msgs.msg import String, Duration

def callback_receive_fn(msg):
	print(msg)
	print("The delay is undesirable in Network")
	




if __name__ == "__main__": 
	rospy.init_node("heart_beat_feedback_node")
	sub = rospy.Subscriber("/heart_beat_feedback", Duration, callback_receive_fn)
	rospy.spin()
