#
#   Misuration SERVER in Python
#   Binds REP socket to tcp://*:5556
#   Executes misuration with sensor and expect calls from client, 
#   replies with the requested measure. 
#

import time
import zmq
import os
import warnings
import socket
import RPi.GPIO as IO 
import numpy as np
import math
from numpy.linalg import norm
import board
from adafruit_icm20x import ICM20948, MagDataRate
import cv2


i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5556")


def IMU():
    # From IMU
    # CAPIRE COME è ORIENTATO, X = 0? Y = 1?Intanto assumo così
    acc=[0,0,0]
    acc[0]=icm.acceleration[0]
    acc[1]=icm.acceleration[1]
    acc[2]=icm.acceleration[2]

    gyr=[0,0,0]
    gyr[0]=icm.gyro[0] 
    gyr[1]=icm.gyro[1] 
    gyr[2]=icm.gyro[2]  #==> questa è la vel lungo Z? 

    return acc




def main():
    while 1:
        # All the measure are performed in loop if requested they
        # are send to the client. 


        print("Wayting for client to connect...")
        #  Wait for next request from client
        message = socket.recv()
        
        if message == b"IMU" :
            #Send in reply the misuration
            accelerometro = IMU()
            socket.send(accelerometro)
            print("Received request: %s" % message)

        #  Do some 'work'
        time.sleep(1)

        #  Send reply back to client
        #socket.send(b"World")



if __name__ == "__main__":
    
    try:
        main()    
    
    except KeyboardInterrupt:
        print("Interrupted")
        os.system("sudo fuser -k 5556/tcp")
        