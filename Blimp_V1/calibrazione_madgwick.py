# Main file for running the Blimp
# it uses all the functions present in "blimp class"

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map
#, kalman_blimp
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


mag_file = open("data_UWB.txt","w")
data_string = ""

#Definizione di tutti i pin per uso
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



##################################################################
# Codice per provare a usare UWB
mesg = {}

#SET OUTPUT MESSAGE
hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
rl = ReadLine(hser)

######################################################

###############################################################
# Codice per stabilire rotazione iniziale del Blimp
# nello spazio e stabilire orientazione dei suoi assi 
# rispetto al sistema di riferimento globale 
#################################################################


def blimp_to_world_rf():
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []
    x = np.empty((1,0))
    y = np.empty((1,0))

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        mesg0 = rl.readline().decode("utf-8")
        #print(mesg)
        ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        #print(dt_uwb)
        tempi_dt[0,:] = dt_uwb

        time_zero = time.perf_counter()
        time_start = time.perf_counter()
        while  time.perf_counter() < (time_start + 20 ):
            raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)
            #Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            # print(magnetometer)

            time_fine = time.perf_counter()
            #setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero = time.perf_counter()
            quat = Quaternion(quaternion)
            #print("res = ", str(quaternion))
            roll, pitch, yaw = quat.to_euler123()  # Result is in rad
            #print(roll, pitch, yaw)
            roll_vec.append(roll)
            pitch_vec.append(pitch)
            yaw_vec.append(yaw)
            
            #Npwm = 50
            #p1.ChangeDutyCycle(Npwm)
            #p2.ChangeDutyCycle(Npwm)
            # Misuro con UWB la posizione nel piano  frattempo 
            mesg = rl.readline().decode("utf-8")
            #print(mesg)
            ts = mesg.split(" ")
            
            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
            print(x_pos_uwb,y_pos_uwb)
            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            for i in range(len(dt_new)) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[i,:], m=2))
            #dt_uwb = np.mean(reject_outliers(tempi_dt))
            #dt_uwb = dt_new
            if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 :
                x = np.append(x, x_pos_uwb)
                y = np.append(y, y_pos_uwb)




            data_string = str(x_pos_uwb) + " ," + str(y_pos_uwb) + " ," + str(z_pos_uwb) + " ," + str(yaw) + "\n"
            mag_file.write(data_string)
            
    
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')

    

    
    #Npwm = 100
    #p1.ChangeDutyCycle(Npwm)
    #p2.ChangeDutyCycle(Npwm)
    quat_final = quat # the last quaternion is stored as input for the madgwick later...
    roll_0 = sum(roll_vec[-40 :])/40 #perform the mean of the last 20 element
    pitch_0 = sum(pitch_vec[-40 :])/40
    yaw_0 = sum(yaw_vec[-40 :])/40
    
    #x = positions[:,0]
    #y = positions[:,1]

    # Ora usando i dati creati vado a fare una LLS estimation
    # y = a +b*x
    a = (np.sum(y) * np.sum(x**2)- np.sum(x)*np.sum(x*y))/ (len(x)*np.sum(x**2) - np.sum(x)**2)
    b = ( len(x)* np.sum(x*y) - np.sum(x)*np.sum(y)) /(len(x)*np.sum(x**2)- np.sum(x)**2) 

    # The error of my model
    

    ang_rad = np.arctan(b) # the inclination of the line calculated in rad with uwb
    delta = yaw_0 -ang_rad


    return delta, ang_rad, quat_final, yaw_0 # These are the values of initial angles
    # This ang_rad is the parameter to insert in both the A* and PID phi as initial orientation calculation!!
    # quat_final is the initial input for Madgwick relaunch
    
    # Per usare la funzione blimp to world correttamente 
    # !! psi_mad_abs =  + psi_mad_relative - delta. # Se mantengo gli angoli anche negativi funziona bene '''

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
        # CHIEDERE CODICE !
    '''try:
        #while True:
        mesg = rl.readline().decode("utf-8")
        print(mesg)


    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')

    #x_pos, y_pos, z_pos_tri = trilateration(d1,d2,d3,d4,d5,d6)
    ts = mesg.split(" ")
    dt_uwb = 1e-3 
    x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
    
    print("coordinate = ", x_pos_uwb, y_pos_uwb, z_pos_uwb)'''

 



    return x_pos, y_pos, z_pos, acc[0], acc[1], acc[2], gyr[2] 


    



    # How the code works for sensors

if __name__ == "__main__":
   
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []

    time_zero = time.perf_counter()
    while  1:
        raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
        #Aquisizione magnetometro e calibrazione dei dati:
        mag = calibration(raw_mag)
        #Creazione vettore input per classe madgwick
        accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
        # print(magnetometer)
        time_fine = time.perf_counter()
        #setting the variable frequency of updateof Madgwick alghorithm
        mad.samplePeriod = time_fine - time_zero
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
        #print(roll, pitch, yaw)
        roll_vec.append(roll)
        pitch_vec.append(pitch)
        yaw_vec.append(yaw)
        


