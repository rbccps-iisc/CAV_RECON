import socket
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

HOST = '10.0.0.2'  # Standard loopback interface address (localhost)
PORT = 12345        # Port to listen on (non-privileged ports are > 1023)

"""
In this test code we are testing basic vehicle control over the network
we use barebone sockets to send the control commands 
This script runs at the on-board vehicle computer. 
"""

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            if not data:
                break
            #date = datetime.datetime.now()
            #conn.sendall(date)
            print(data)
            words = data.decode('utf-8').split(",")
            for i in words:
                print(i)
                ser.write(i.encode('utf-8'))