# Calibration for the psi_map references

import warnings
import socket
#import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map, imu_to_uwb
import numpy as np
import math
from numpy.linalg import norm
from quaternion import Quaternion
#import serial
import time
import select
import threading
#import board
#from adafruit_icm20x import ICM20948, MagDataRate
import zmq
import global_
import logging

# Definizione context per zmq
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://192.168.1.104:5556")
print("Mi sono connesso al Raspberry pi Blimp")

# Subscribe to zipcode, default is Blimp
zip_code = "Blimp"
socket.setsockopt_string(zmq.SUBSCRIBE, zip_code)

def str_to_float():
            message2 = socket.recv_string()
            zip_code, tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, mesg, zpos = message2.split(",")
            tempo = float(tempo)
            raw_accX = float(raw_accX)
            raw_accY = float(raw_accY)
            raw_accZ = float(raw_accZ)
            raw_gyrX = float(raw_gyrX)
            raw_gyrY = float(raw_gyrY)
            raw_gyrZ = float(raw_gyrZ)
            raw_magX = float(raw_magX)
            raw_magY = float(raw_magY)
            raw_magZ = float(raw_magZ)
            zpos = float(zpos)
            return tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg

if __name__ == "__main__":
   
    # q0 = Quaternion(0, 0, 0, 1)
    q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.09, quaternion=q0, beta=0.1)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []

    tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
    ts = mesg0.split(" ")
    if (len(ts)!=25):
      tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
      ts = mesg0.split(" ")

    time_zero = tempo

    time_start = time.perf_counter()

    while  1:

        # Misuro con UWB la posizione nel piano  frattempo 
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
        ts = mesg.split(" ")
        if (len(ts)!=25):
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
            ts = mesg.split(" ")   

        if (time.perf_counter() - time_start) >= 0.09:
            
            raw_acc = [raw_accX, raw_accY, raw_accZ]
            raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
            raw_mag = [raw_magX, raw_magY, raw_magZ] 

            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)

            raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag)

            #Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            # print(magnetometer)
            #setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = tempo - time_zero
            # print(1/(time_fine - time_zero))
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero = tempo
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
        


