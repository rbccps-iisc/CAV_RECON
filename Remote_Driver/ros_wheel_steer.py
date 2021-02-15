import pygame
import rospy
from std_msgs.msg import String
import time

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

first_02p = 1
first_04p = 1
first_06p = 1
first_02n = 1
first_04n = 1
first_06n = 1
prev_steer = 0
pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()

calib_throttle = True
calib_brake = True

def talker():
    global first_02p, first_04p, first_06p, first_02n, first_04n, first_06n
    global prev_steer

    pub_steering = rospy.Publisher("steering", String, queue_size=10)
    rospy.init_node('vehicle', anonymous=True)
    while not rospy.is_shutdown():
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.JOYAXISMOTION:
                steer = pygame.joystick.Joystick(0).get_axis(0)
                print("Steer {}     prev_steer: {}".format(steer, prev_steer))
                # Neutral position of the steering
                if -0.015 < steer < 0.015:
                    prev_steer = 0
                    pub_steering.publish("3000S".encode('utf-8'))

                # Condition checks for the clockwise rotation of the steering
                elif (steer > 0.020) and (steer < 0.040) and first_02p:
                    print("Positive 02")
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_02p = 0
                elif steer > 0.040 and (steer < 0.060) and first_04p:
                    print("Positive 04")
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_04p = 0
                elif steer > 0.060 and (steer < 0.080) and first_06p:
                    print("Positive 06")
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_06p = 0
                elif (prev_steer > 0.060) and (steer < 0.060):
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_06p = 1
                elif (prev_steer > 0.040) and (steer < 0.040):
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_04p = 1
                elif (prev_steer > 0.020) and (steer < 0.020):
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_02p = 1

                # Condition checks for the anti-clockwise rotation of the steering
                elif (steer < -0.020) and (steer > -0.040) and first_02n:
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_02n = 0
                elif (steer < -0.040) and (steer > -0.060) and first_04n:
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_04n = 0
                elif (steer < -0.060) and (steer > -0.080) and first_06n:
                    pub_steering.publish("1000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_06n = 0
                elif (prev_steer < -0.060) and (steer > -0.060):
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_06n = 1
                elif (prev_steer < -0.040) and (steer > -0.040):
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_04n = 1
                elif (prev_steer < -0.020) and (steer > -0.020):
                    pub_steering.publish("2000S".encode('utf-8'))
                    time.sleep(0.10)
                    pub_steering.publish("3000S".encode('utf-8'))
                    prev_steer = steer
                    first_02n = 1

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

