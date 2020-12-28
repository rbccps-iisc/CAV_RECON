#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import serial
from sys import platform
if platform == "linux" or platform == "linux2":
    ser = serial.Serial('/dev/ttyACM0')
elif platform == "darwin":
    pass
elif platform == "win32":
    # Windows...
    ser = serial.Serial('COM16')
"""
In this test code we are testing basic vehicle control over the network
we use ROS middleware to send the control commands 
This script runs at the remote driver end. 
Receives joystick messages (subscribed to Joy topic)
then converts the joystick inputs into commands

WE ARE NOT USING THIS METHOD NOW 
--- WE HAVE SEPERATED OUT ALL THE STREAMS FROM THE JOYSTICK

"""

oldvar = 0
first_a = 0
first_d = 0
# Configuatrion tuned for CAR in LOW speed
base_throttle = 5500
peak_throttle = 6500
base_brake = 450
peak_brake = 600
button = 0


def callback(data):
    global first_a
    global first_d
    global oldvar
    global base_throttle
    global peak_throttle
    global base_brake
    global peak_brake
    global button
    # print data
    axis1 = -data.axes[1]
    axis3 = -data.axes[3]  # in logitech axis 3 is axis 4 confirm with ashish
    button1 = data.buttons[1]
    button4 = data.buttons[4]
    button5 = data.buttons[5]

    button_ = button1+button4+button5

    if axis1 > 0.1:
        bval = int((axis1) * (peak_brake - base_brake) + base_brake)
        print(bval)
        ser.write(str(bval).encode('utf-8'))
        ser.write("a".encode('utf-8'))
        #### ser.write("4000a".encode('utf-8')) #throttle released on braking
        print("Brake")
    elif (axis1 < -0.1 and axis3 < 0.1):
        tval = int((axis1 * -1 + axis3 * -1) * (peak_throttle - base_throttle) * 0.5 + base_throttle)
        if (abs(tval - oldvar) > 5):
            #print(tval)
            ser.write(str(tval).encode('utf-8'))
            ser.write("a".encode('utf-8'))
            ser.write("450a".encode('utf-8'))  # brake released on acceleration
            print("Throttle")
        oldvar = tval
    elif (axis1 > -0.1 and axis1 < 0.1):
        ser.write("4000a".encode('utf-8'))
        ser.write("450a".encode('utf-8'))  # brake released
        print("Zero Throttle")
    print (axis1)
    print (axis3)

    if button1 == 1:
        print("Emergency Brake")
        ser.write("4600a".encode('utf-8'))  # throttle released
        ser.write("600a".encode('utf-8'))  # brake engaged

    if (button4 and button5 == 0):
        if (first_a == 0):
            ser.write("1000a".encode('utf-8'))
            print("Joystick button 4 pressed.")
            first_a = 1
    if (button5 and button4 == 0):
        if (first_d == 0):
            ser.write("2000a".encode('utf-8'))
            print("Joystick button 5 pressed.")
            first_d = 1

    if(button-button_!= 0):
        if(button4 == 0):
            first_a = 0
        if(button5 == 0):
            first_d = 0
        ser.write("3000a".encode('utf-8'))
        print("Joystick button released.")
    button = button_

# Intializes everything
def start():
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()


if __name__ == '__main__':
    start()
