#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Time, Duration


if __name__ == "__main__":
    rospy.init_node("Heart_beat_publisher", anonymous=True)
    pub = rospy.Publisher("/heart_beat", Time, queue_size = 10)
    rate = rospy.Rate(1)
    
    while not rospy.is_shutdown():
        msg = Time() 
        pub_time = rospy.Time.now()
        msg.data = pub_time
        print(msg)
        pub.publish(msg)
        rate.sleep()
    rospy.loginfo("Node has Stopped")		
