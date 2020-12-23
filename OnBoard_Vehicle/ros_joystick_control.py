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

oldvar = 0
first_a = 0
first_d = 0
target = ""
oldtarget = ""
base_throttle = 5500
peak_throttle = 6500
base_brake = 450
peak_brake = 600
bval = 0
button = 0

def callback(data):
    global first_a, target, oldtarget
    global first_d
    global oldvar
    global base_throttle
    global peak_throttle
    global base_brake
    global peak_brake
    global bval
    global button

    axis1 = -data.axes[1]
    axis3 = -data.axes[4]  # in logitech axis 3 is axis 4 confirm with ashish

    button1 = data.buttons[1]
    button4 = data.buttons[4]
    button5 = data.buttons[5]

    button_ = button1+button4+button5

    if axis1 > 0.1:
        bval = axis1
        if 0.1 < bval < 0.2:
            target = "200B"
        elif 0.3 < bval < 0.4:
            target = "220B"
        elif 0.4 < bval < 0.5:
            target = "240B"
        elif 0.5 < bval < 0.6:
            target = "260B"
        elif 0.6 < bval < 0.7:
            target = "280B"
        elif 0.7 < bval < 0.8:
            target = "300B"
        elif 0.9 < bval < 1:
            target = "320B"
        if target != oldtarget:
            ser.write(target.encode('utf-8'))
            # t1 = datetime.datetime.now()
            oldtarget = target
            print(target)
    elif axis1 < -0.1 and axis3 < 0.1:
        tval = axis1 * -2  # + axis3 * -1
        if 0.1 < tval < 0.2:
            target = "0T"
        elif 0.2 < tval < 0.3:
            target = "10T"
        elif 0.3 < tval < 0.4:
            target = "20T"
        elif 0.4 < tval < 0.5:
            target = "30T"
        elif 0.5 < tval < 0.6:
            target = "40T"
        elif 0.6 < tval < 0.7:
            target = "50T"
        elif 0.7 < tval < 0.8:
            target = "60T"
        elif 0.8 < tval < 0.9:
            target = "70T"
        elif 0.9 < tval < 1:
            target = "80T"
        elif 1 < tval < 1.1:
            target = "90T"
        elif 1.1 < tval < 1.2:
            target = "100T"
        elif 1.2 < tval < 1.3:
            target = "110T"
        elif 1.3 < tval < 1.4:
            target = "120T"
        elif 1.4 < tval < 1.5:
            target = "130T"
        elif 1.5 < tval < 1.6:
            target = "140T"
        elif 1.6 < tval < 1.7:
            target = "150T"
        elif 1.7 < tval < 1.8:
            target = "160T"
        elif 1.9 < tval < 2:
            target = "160T"
        if target != oldtarget:
            ser.write(target.encode('utf-8'))

            ser.write("0T".encode('utf-8'))
            oldtarget = target
            print(target)
    elif -0.1 < axis1 < 0.1:
        print("Zero Throttle")
        ser.write("200B".encode('utf-8'))
        ser.write("0T".encode('utf-8'))

    if button1 == 1:
        print("Emergency Brake")
        ser.write("320B".encode('utf-8'))
        ser.write("0T".encode('utf-8'))
    if (button4 and button5 == 0):
        if (first_d == 0):
            ser.write("1000S".encode('utf-8'))
            print("Joystick button 4 pressed.")
            first_a = 1
    if (button5 and button4 == 0):
        if (first_d == 0):
            ser.write("2000S".encode('utf-8'))
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
    rospy.init_node('Joy2Vehicle')
    rospy.spin()