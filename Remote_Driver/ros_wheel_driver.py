import pygame
import time
import serial
import rospy
from std_msgs.msg import String

"""
In this test code we are testing basic vehicle control over the network
we use ROS middleware to send the control commands 
This script runs at the remote driver end. 
Receives logitech wheel messages and publish to topics, 
1. steering
2. break
3. throttle 

Logitech wheel axis 2 = Accelerator
Logitech wheel axis 3 = Brake
Logitech wheel axis 0 = Steering
Logitech wheel button 6 = Emergency Brake

"""

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
pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()
#ser = serial.Serial('/dev/ttyUSB0')


def talker():
    global first_a, target, oldtarget
    global first_d
    global oldvar
    global base_throttle
    global peak_throttle
    global base_brake
    global peak_brake
    global bval

    pub_break = rospy.Publisher('break', String, queue_size=10)
    pub_throttle = rospy.Publisher('throttle', String, queue_size=10)
    pub_steering = rospy.Publisher("steering", String, queue_size=10)
    rospy.init_node('vehicle', anonymous=True)
    while not rospy.is_shutdown():
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.JOYAXISMOTION:
                steer = pygame.joystick.Joystick(0).get_axis(0)
                accel = pygame.joystick.Joystick(0).get_axis(2)
                brake = pygame.joystick.Joystick(0).get_axis(3)

                print("Steer: {}	Brake: {}	Accel: {}".format(steer, brake, accel))
                if brake < 0.95:
                    bval = brake
                    if 0.95 > bval > 0.714:
                        target = "240B"
                    elif 0.714 > bval > 0.428:
                        target = "260B"
                    elif 0.428 > bval > 0.142:
                        target = "280B"
                    elif 0.142 > bval > -0.142:
                        target = "300B"
                    elif -0.142 > bval > -0.428:
                        target = "310B"
                    elif -0.428 > bval > -0.714:
                        target = "320B"
                    elif -0.714 > bval > -1.0:
                        target = "320B"
                    if target != oldtarget:
                        #ser.write(target.encode('utf-8'))
                        pub_break.publish(target.encode('utf-8'))
                        oldtarget = target
                        print(target)

                # elif brake > 0.95 and accel < 0.95:
                elif accel < 0.95:
                    # tval = axis1 * -2 #+ axis3 * -1
                    tval = accel
                    if 0.95 > tval > 0.8:
                        target = "0T"
                    elif 0.8 > tval > 0.7:
                        target = "10T"
                    elif 0.7 > tval > 0.6:
                        target = "20T"
                    elif 0.6 > tval > 0.5:
                        target = "30T"
                    elif 0.5 > tval > 0.4:
                        target = "40T"
                    elif 0.4 > tval > 0.3:
                        target = "50T"
                    elif 0.3 > tval > 0.2:
                        target = "60T"
                    elif 0.2 > tval > 0.1:
                        target = "70T"
                    elif 0.1 > tval > 0.0:
                        target = "70T"
                    elif 0.0 > tval > -0.1:
                        target = "70T"
                    elif -0.1 > tval > -0.2:
                        target = "70T"
                    elif -0.2 > tval > -0.3:
                        target = "70T"
                    elif -0.3 > tval > -0.4:
                        target = "70T"
                    elif -0.4 > tval > -0.5:
                        target = "70T"
                    elif -0.5 > tval > -0.6:
                        target = "70T"
                    elif -0.6 > tval > -0.7:
                        target = "70T"
                    elif -0.7 > tval > -0.8:
                        target = "70T"
                    elif -0.9 > tval > -1.0:
                        target = "70T"
                    if target != oldtarget:
                        #ser.write(target.encode('utf-8'))
                        #qser.write("0T".encode('utf-8'))
                        pub_throttle.publish(target.encode('utf-8'))
                        oldtarget = target
                        print(target)

                elif steer > 0.040:
                    if first_d == 0:
                        pub_steering.publish("2000S".encode('utf-8'))
                        print("2000S")
                        first_d = 1
                elif steer < -0.040:
                    if first_a == 0:
                        pub_steering.publish("1000S".encode('utf-8'))
                        print("1000S")
                        first_a = 1
                elif -0.040 < steer < -0.020:
                    pub_steering.publish("3000S".encode('utf-8'))
                    print("3000S")
                    first_a = 0
                elif 0.020 < steer < 0.040:
                    pub_steering.publish("3000S".encode('utf-8'))
                    print("3000S")
                    first_d = 0

                elif 0.97 < accel < 0.95:
                    print("Zero Throttle")
                    #ser.write("200B".encode('utf-8'))
                    #ser.write("0T".encode('utf-8'))
                    pub_break.publish("200B".encode('utf-8'))
                    pub_throttle.publish("0T".encode('utf-8'))

            if event.type == pygame.JOYBUTTONDOWN:
                if pygame.joystick.Joystick(0).get_button(6):
                    print("Emergency Brake")
                    #ser.write("320B".encode('utf-8'))
                    #ser.write("0T".encode('utf-8'))
                    pub_break.publish("320B".encode('utf-8'))
                    pub_throttle.publish("0T".encode('utf-8'))

            #     if pygame.joystick.Joystick(0).get_button(4) and pygame.joystick.Joystick(0).get_button(5) == 0:
            #         if first_a == 0:
            #             #ser.write("1000S".encode('utf-8'))
            #             pub_steering.publish("1000S".encode('utf-8'))
            #             print("Joystick button 4 pressed.")
            #             first_a = 1
            #     if pygame.joystick.Joystick(0).get_button(5) and pygame.joystick.Joystick(0).get_button(4) == 0:
            #         if first_d == 0:
            #             #ser.write("2000S".encode('utf-8'))
            #             pub_steering.publish("2000S".encode('utf-8'))
            #             print("Joystick button 5 pressed.")
            #             first_d = 1
            # elif event.type == pygame.JOYBUTTONUP:
            #     if pygame.joystick.Joystick(0).get_button(4) == 0:
            #         first_a = 0
            #     if pygame.joystick.Joystick(0).get_button(5) == 0:
            #         first_d = 0
            #     #ser.write("3000S".encode('utf-8'))
            #     pub_steering.publish("3000S".encode('utf-8'))
            #     print("Joystick button released.")


if __name__ == '__main__':
    talker()

