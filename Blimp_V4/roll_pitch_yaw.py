import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration
import numpy as np
import math
from numpy.linalg import norm, inv
from quaternion import Quaternion
import serial
import time
#import select
#import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate
#import cv2
import logging
#import zmq



'''
# Info per i messaggi
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://192.168.1.176:5556")
'''
         
i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ



if __name__ == "__main__":
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
    
    time_zero_mad = time.perf_counter()

    zero = time.perf_counter()
    while True:
        
        raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
        # Aquisizione magnetometro e calibrazione dei dati:
        mag = calibration(raw_mag)
        # Creazione vettore input per classe madgwick
        accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)

        # setting the variable frequency of update of Madgwick alghorithm
        mad.samplePeriod = time.perf_counter() - time_zero_mad
        quaternion = mad.update(gyroscope, accelerometer, magnetometer)
        time_zero_mad = time.perf_counter()
        quat = Quaternion(quaternion)
        
        roll, pitch, yaw = quat.to_euler123()  # Result is in rad

        if (time.perf_counter() - zero) >= 1.0:
            print(roll*180/np.pi, pitch*180/np.pi, yaw*180/np.pi)
            zero = time.perf_counter()
        