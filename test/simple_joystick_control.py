import pygame
import time
import json

oldvar = 0
first_a = 0
first_d =0
#Configuatrion tuned for CAR in LOW speed
base_throttle=5500
peak_throttle=6500
base_brake=450
peak_brake=600
pygame.display.init()
pygame.joystick.init()
pygame.joystick.Joystick(0).init()

def remote():
    global first_a
    global first_d
    global oldvar
    global base_throttle
    global peak_throttle
    global base_brake
    global peak_brake

    while 1:
        out_json = {
            "throttle": "",
            "break": "",
            "emergency": False,
            "steerL": "",
            "steerR": "",
            "modified": False
        }
        for event in pygame.event.get():  # User did something.
            if event.type == pygame.JOYAXISMOTION:
                axis1 = pygame.joystick.Joystick(0).get_axis(1)
                axis3 = pygame.joystick.Joystick(0).get_axis(3)
                if (axis1 > 0.1):
                    bval = int((axis1) * (peak_brake - base_brake) + base_brake)
                    print(bval)
                    out_json["break"]=bval
                    out_json["modified"]=True
                elif (axis1 < -0.1 and axis3 < 0.1):
                    tval = int((axis1 * -1 + axis3 * -1) * (peak_throttle - base_throttle) * 0.5 + base_throttle)
                    if (abs(tval - oldvar) > 5):
                        print(tval)
                        out_json["throttle"]=tval
                        out_json["modified"] = True
                    oldvar=tval
                elif(axis1>-0.1 and axis1<0.1):
                    print("Zeron Throttle")
                    out_json["throttle"] = 0
                    out_json["modified"] = True
                print (axis1)
                print (axis3)
            if event.type == pygame.JOYBUTTONDOWN:
                if (pygame.joystick.Joystick(0).get_button(1)):
                    print("Emergency Brake")
                    out_json["emergency"]=True
                    out_json["modified"] = True
                if (pygame.joystick.Joystick(0).get_button(4) and pygame.joystick.Joystick(0).get_button(5) == 0):
                    if (first_a == 0):
                        print("Joystick button 4 pressed.")
                        first_a = 1
                        out_json["steerL"]=first_a
                        out_json["modified"] = True
                if (pygame.joystick.Joystick(0).get_button(5) and pygame.joystick.Joystick(0).get_button(4) == 0):
                    if (first_d == 0):
                        print("Joystick button 5 pressed.")
                        first_d = 1
                        out_json["steerR"]=first_d
                        out_json["modified"] = True
            elif event.type == pygame.JOYBUTTONUP:
                if (pygame.joystick.Joystick(0).get_button(4) == 0):
                    first_a = 0
                    out_json["steerL"] = first_a
                    out_json["modified"] = True
                if (pygame.joystick.Joystick(0).get_button(5) == 0):
                    first_d = 0
                    out_json["steerR"]=first_d
                    out_json["modified"] = True
                print("Joystick button released.")

        return out_json

if __name__ == '__main__':
    remote()
    


