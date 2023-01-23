# Main file for running the Blimp
# it uses all the functions present in "blimp class"

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, orientation_initial, PID_Controller, rad_to_deg  #, kalman_blimp
import numpy as np
import math
from numpy.linalg import norm
from quaternion import Quaternion
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate


#Definizione di tutti i pin per uso
m1 = 20
m1_i = 21
m2 = 13
m2_i = 19
m3 = 17
m3_i = 27

HCtrig=23     
HCecho=24     

IO.setwarnings(False)           
IO.setmode(IO.BCM)        #numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori     
IO.setup(HCtrig,IO.OUT)   # setup ultrasuoni
IO.setup(HCecho,IO.IN)  

                    
i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ

# Function to call for get measurement of all the sensors togheter
def misuration():
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

    # From Sonar
    # Set Trigger to High
    IO.output(HCtrig, True)
    # Set trigger after 0.01 ms to Low
    time.sleep(0.00001)
    IO.output(HCtrig, False)

    StartTime = time.time()
    StopTime = time.time()

    while IO.input(HCecho)==0:
        StartTime = time.time()
    
    while IO.input(HCecho)==1:
        StopTime = time.time()
    
    pulse_duration = StopTime - StartTime
    distance = pulse_duration * 171.50 #result in meter
    z_pos = round(distance, 3)
    

    #From UWB Absolute Reference frame, meters
        # CHIEDERE CODICE !!
    x_pos = 0.
    y_pos = 0.

    



    return x_pos, y_pos, z_pos, acc[0], acc[1], acc[2], gyr[2] 



    # How the code works for sensors

if __name__ == "__main__":
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    #Madgwick initialization
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1) 

    # SET ZERO TIME, FUNDAMENTAL BEFORE THE WHILE LOOP
    time_zero = time.perf_counter()
while 1: 
    val = 0
    #Aquisizione magnetometro e calibrazione dei dati:
    mag = calibration(icm.magnetic)
    #Creazione vettore input per classe madgwick
    accelerometer, gyroscope, magnetometer = np.asarray(icm.acceleration), np.asarray(icm.gyro), np.asarray(mag)
    time_fine = time.perf_counter()
    #setting the variable frequency of updateof Madgwick alghorithm
    mad.samplePeriod = time_fine - time_zero
    quaternion = mad.update(gyroscope, accelerometer, magnetometer)
    time_zero = time.perf_counter()
    quat = Quaternion(quaternion)
    roll, pitch, yaw = quat.to_euler123()
    phi = yaw #measured orientation
    

    # For visualization on terminal only! 
    rolldeg, pitchdeg, yawdeg = rad_to_deg(roll, pitch, yaw)
    #Other sensors
    pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel = misuration()
    #print(pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel)
    print('yaw = ', yawdeg )

   
