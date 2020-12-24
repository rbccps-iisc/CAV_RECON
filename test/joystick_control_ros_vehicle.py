import serial
import datetime
from sys import platform

if platform == "linux" or platform == "linux2":
    ser = serial.Serial('/dev/ttyUSB0')
elif platform == "darwin":
    pass
elif platform == "win32":
    # Windows...
    ser = serial.Serial('COM6')


import rospy
from std_msgs.msg import String

def callback(target):
    print(target)
    ser.write((target.data).encode('utf-8'))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('break', String, callback)
    rospy.Subscriber('throttle', String, callback)
    rospy.Subscriber('steering', String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()

