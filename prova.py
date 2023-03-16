
# Main file for running the Blimp
# it uses all the functions present in "blimp class"

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, PID_Controller, trilateration, Astar #, kalman_blimp
import numpy as np
import math
from numpy.linalg import norm
from quaternion import Quaternion
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate
import cv2


#Definizione di tutti i pin per uso
m1 = 17 #sinistro
m1_i = 27
m2 = 13  #destro
m2_i = 19
m3 =  20 #sotto
m3_i =  21

HCtrig=23     
HCecho=24     

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

# Codice per stabilire rotazione iniziale del Blimp
# nello spazio e stabilire orientazione dei suoi assi 
# rispetto al sistema di riferimento globale 

def blimp_to_world_rf(raw_acc, raw_gyr, raw_mag):
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []
    x = [0.0] #Nella prova vera qui devi togliere questi valori
    y = [0.0]

    time_zero = time.perf_counter()
    time_start = time.perf_counter()
    with open('yaw_blimp_to_world.txt', 'w') as f:
        while  time.perf_counter() < (time_start +10):
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)
            #Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            time_fine = time.perf_counter()
            #setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero = time.perf_counter()
            quat = Quaternion(quaternion)
            #print("res = ", str(quaternion))
            roll, pitch, yaw = quat.to_euler123()  # Result is in rad
            print(roll, pitch, yaw)
            roll_vec.append(roll)
            pitch_vec.append(pitch)
            yaw_vec.append(yaw)
            # Write in the file
            f.write(str(yaw))
            f.write('\n')

            Npwm = 50
            #p1.ChangeDutyCycle(Npwm)
            #p2.ChangeDutyCycle(Npwm)
            # Misuro con UWB la posizione nel frattempo 
            x_pos, y_pos, z_pos, acc_x, acc_y, acc_z, gyr = misuration()
            #positions = np.append(positions, [x_pos, y_pos])
            x = np.append(x, x_pos)
            y = np.append(y, y_pos)

    Npwm = 0
    #p1.ChangeDutyCycle(Npwm)
    p2.ChangeDutyCycle(Npwm)
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

    ang_rad = np.arctan(b) # the inclination of the line calculated in rad with uwb
    delta = np.abs(ang_rad-yaw_0)


    return delta, ang_rad, quat_final, yaw_0 # These are the values of initial angles
    # This ang_rad is the parameter to insert in both the A* and PID phi as initial orientation calculation!!

    
# Per usare la funzione blimp to world correttamente 
# phi_mad_abs = delta + phi_mad_relative . # Se mantengo gli angoli anche negativi funziona bene 


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

    d1 = 6.40311549 
    d2= 3.5336808
    d3 =  1.82277261 
    d4= 4.43949321
    d5 = 5.95727287 
    d6 = 8.8512711 

    x_pos, y_pos, z_pos_tri = trilateration(d1,d2,d3,d4,d5,d6)
    

    



    return x_pos, y_pos, z_pos, acc[0], acc[1], acc[2], gyr[2] 



    # How the code works for sensors
import timeit
if __name__ == "__main__":
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    # Primo step, calcolo della orientazione iniziale del 
    # dirigibile nello spazio
    delta, ang_rad, quat_final, yaw_0 = blimp_to_world_rf(icm.acceleration,icm.gyro, icm.magnetic) #==> delta, ang rad
     # Plotting the final results
    print("delta = ", delta*180/np.pi)
    print("ang calcolato = ", ang_rad*180/np.pi)
    print("quat_final = ", quat_final*180/np.pi)
    print("yaw_0 = ", yaw_0*180/np.pi)

    # Path Planning prova
    print("Inizio path planning")
    img = cv2.flip(cv2.imread("C:\Volume_D\Programming\Blimp_git\Blimp\povo2_provaPathPlanning.png"),0)
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20, 20)))
    img = img.astype(float)/255.
    
    start = (150,150) ## MY case
    goal = (360, 760)

    a = timeit.default_timer()
    astar = Astar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=10) #10 è 1 metro nella realtà
    print(path)
    b = timeit.default_timer()
    print("Time: ", b-a)


    