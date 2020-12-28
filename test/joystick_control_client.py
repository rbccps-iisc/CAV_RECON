import pygame
import time
#import serial
import socket
import datetime

"""
In this test code we are testing basic vehicle control over the network
we use barebone sockets to send the control commands 
This script runs at the remote driver end. 
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
#ser = serial.Serial('COM6')
s = socket.socket()

HOST = '10.0.0.2'
PORT = 12345

SOCK = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

SOCK.connect((HOST, PORT))

def talker():
    global first_a, target, oldtarget
    global first_d
    global oldvar
    global base_throttle
    global peak_throttle
    global base_brake
    global peak_brake
    global bval
    while 1:
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.JOYAXISMOTION:
                axis1 = pygame.joystick.Joystick(0).get_axis(1)
                axis3 = pygame.joystick.Joystick(0).get_axis(3)
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
                        #ser.write(target.encode('utf-8'))
                        #t1 = datetime.datetime.now()
                        SOCK.sendall(target.encode('utf-8'))
                        #date = SOCK.recv(1024)
                        #print (date)
                        oldtarget = target
                        print(target)
                elif axis1 < -0.1 and axis3 < 0.1:
                    tval = axis1 * -2 #+ axis3 * -1
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
                        #ser.write(target.encode('utf-8'))
                        SOCK.sendall(target.encode('utf-8'))
                        # qser.write("0T".encode('utf-8'))
                        oldtarget = target
                        print(target)
                elif -0.1 < axis1 < 0.1:
                    print("Zero Throttle")
                    #ser.write("200B".encode('utf-8'))
                    SOCK.sendall("200B".encode('utf-8')+",")
                    #ser.write("0T".encode('utf-8'))
                    SOCK.sendall("0T".encode('utf-8'))
            if event.type == pygame.JOYBUTTONDOWN:
                if pygame.joystick.Joystick(0).get_button(1):
                    print("Emergency Brake")
                    #ser.write("320B".encode('utf-8'))
                    SOCK.sendall("320B".encode('utf-8')+",")
                    #ser.write("0T".encode('utf-8'))
                    SOCK.sendall("0T".encode('utf-8'))
                if pygame.joystick.Joystick(0).get_button(4) and pygame.joystick.Joystick(0).get_button(5) == 0:
                    if first_a == 0:
                        #ser.write("1000S".encode('utf-8'))
                        SOCK.sendall("1000S".encode('utf-8'))
                        print("Joystick button 4 pressed.")
                        first_a = 1
                if pygame.joystick.Joystick(0).get_button(5) and pygame.joystick.Joystick(0).get_button(4) == 0:
                    if first_d == 0:
                        #ser.write("2000S".encode('utf-8'))
                        SOCK.sendall("2000S".encode('utf-8'))
                        print("Joystick button 5 pressed.")
                        first_d = 1
            elif event.type == pygame.JOYBUTTONUP:
                if pygame.joystick.Joystick(0).get_button(4) == 0:
                    first_a = 0
                if pygame.joystick.Joystick(0).get_button(5) == 0:
                    first_d = 0
                #ser.write("3000S".encode('utf-8'))
                SOCK.sendall("3000S".encode('utf-8'))
                print("Joystick button released.")


if __name__ == '__main__':
    talker()
