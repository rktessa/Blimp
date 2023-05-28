# Calibration for the psi_map references

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map
import numpy as np
import math
from numpy.linalg import norm
from quaternion import Quaternion
import serial
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate

#Definizione di tutti i pin per uso
m1 = 17 #sinistro
m1_i = 27
m2 = 13  #destro
m2_i = 19
m3 =  20 #sotto
m3_i =  21

HCtrig=8     
HCecho=7     

IO.setwarnings(False)           
IO.setmode(IO.BCM)        #numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori  
IO.setup(m1_i,IO.OUT)
IO.setup(m2,IO.OUT)
IO.setup(m2_i,IO.OUT)
IO.setup(m3,IO.OUT)
IO.setup(m3_i,IO.OUT)  
IO.setup(HCtrig,IO.OUT)   # setup ultrasuoni
IO.setup(HCecho,IO.IN)   

p1 = IO.PWM(m1,100)       #inizializzazione pwm        
p1i = IO.PWM(m1_i,100) 
p1.start(0)
p1i.start(0)     
p2 = IO.PWM(m2,100)
p2i = IO.PWM(m2_i,100)             
p2.start(0)
p2i.start(0)   
p3 = IO.PWM(m3,100)
p3i = IO.PWM(m3_i,100)             
p3.start(0)
p3i.start(0)                      
i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ





if __name__ == "__main__":
   
    # q0 = Quaternion(0, 0, 0, 1)
    q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.5, quaternion=q0, beta=0.1)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []

    time_zero = time.perf_counter()

    time_start = time.perf_counter()

    while  1:

        if (time.perf_counter() - time_start) >= 0.5:
            raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)
            #Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            # print(magnetometer)
            time_fine = time.perf_counter()
            #setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            # print(1/(time_fine - time_zero))
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero = time.perf_counter()
            quat = Quaternion(quaternion)
            #print("res = ", str(quaternion))
            roll, pitch, yaw = quat.to_euler123()  # Result is in rad

            psi = yaw*180/np.pi
            if psi < 0:
                psi = psi+ 360
            
            #print("psi = ", psi)
            
            psi_mapped = psi_map(psi)
            print("psi_mapped = ", psi_mapped, "  ", "psi = ", psi)
            
            roll_vec.append(roll)
            pitch_vec.append(pitch)
            yaw_vec.append(yaw)

            time_start = time.perf_counter()
        


