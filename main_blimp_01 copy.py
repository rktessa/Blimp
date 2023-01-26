# Main file for running the Blimp
# it uses all the functions present in "blimp class"

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, PID_Controller, trilateration #, kalman_blimp
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
icm = adafruit_icm20x.ICM20948(i2c)

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
    

    time_zero = time.perf_counter()
    time_start = time.perf_counter()
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
        if time.perf_counter() > (time_start +7):
            Npwm = 50
            p1.ChangeDutyCycle(Npwm)
            p2.ChangeDutyCycle(Npwm)
            # Misuro con UWB la posizione nel frattempo 
            x_pos, y_pos, z_pos, acc_x, acc_y, acc_z, gyr = misuration()
            positions = np.append(positions, [x_pos, y_pos])


    
    quat_final = quat # the last quaternion is stored as input for the madgwick later...
    roll_0 = sum(roll_vec[-40 :])/40 #perform the mean of the last 20 element
    pitch_0 = sum(pitch_vec[-40 :])/40
    yaw_0 = sum(yaw_vec[-40 :])/40
    
    x = positions[:,0]
    y = positions[:,1]

    # Ora usando i dati creati vado a fare una LLS estimation
    # y = a +b*x
    a = (np.sum(y) * np.sum(x**2)- np.sum(x)*np.sum(x*y))/ (len(x)*np.sum(x**2) - np.sum(x)**2)
    b = ( len(x)* np.sum(x*y) - np.sum(x)*np.sum(y)) /(len(x)*np.sum(x**2)- np.sum(x)**2) 

    ang_rad = np.arctan(b) # the inclination of the line calculated in rad
    delta = np.abs(ang_rad-yaw_0)


    return delta, ang_rad, quat_final # These are the values of initial angles
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
        # CHIEDERE CODICE !

    x_pos, y_pos, z_pos_tri = trilateration(d1,d2,d3,d4,d5,d6)

    



    return x_pos, y_pos, z_pos, acc[0], acc[1], acc[2], gyr[2] 



    # How the code works for sensors

if __name__ == "__main__":
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    # Primo step, calcolo della orientazione iniziale del 
    # dirigibile nello spazio
    delta, ang_rad, quat_final = blimp_to_world_rf(icm.acceleration,icm.gyro, icm.magnetic) #==> delta, ang rad

    # Calcolare la traiettoria da seguire con il path planner

    #Madgwick initialization
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1) 
    
    # PID initialization
    dist_pid = PID_Controller(kp= 1, ki = 1, kd = 1)
    phi_pid = PID_Controller(kp= 1, ki = 1, kd = 1)
    z_pid = PID_Controller(kp= 1, ki = 1, kd = 1)

    traj = np.array([[0, 0, 1],
                     [1, 1, 1],
                     [2, 2, 2]])
    
    # 


    # Mantenimento di una data quota
    z_target = 1.5 # metri
    psi_target = np.pi/2.0 #rad
    point_target = np.array([5.0, 2.0]) #array of coordinates x,y  m
    dist_target = 0.01 #m distance the blimp want to minimize
    # Kalman initialization
    # kal = kalman_blimp()


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
    #OtHer sensors
    pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel = misuration()
    

    # Z_PID block of code 
    z_pid.set_new_target(z_target)
    #z_pid.sample_rate = differenza di tempo da capire
    signal_z = z_pid.adjust_signal(pos_z)
    force_z =z_pid.get_force_z(signal_z) # Questa serve per il Kalman
    #kal.Fu = force_z
    z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori
    #if Npwm_z >= 0:
        # p1.ChangeDutyCycle(Npwm_z)
    #else:
        #p1i.ChangeDutyCycle(-Npwm_z)
    ###############################
    ###############################
    # dist and phi pid block of code
    # dist_init = 
    dist_dist = math.sqrt((pos_x-point_target[0])**2 + (pos_y - point_target[1])**2)
    phi_pid.set_new_target(psi_target)
    dist_pid.set_new_target(dist_target)
    #phi_pid.sample_rate = differenza di tempo da capire
    #dist_pid.sample_rate = differenza di tempo da capire
    signal_phi = phi_pid.adjust_signal(phi)
    signal_dist = dist_pid.adjust_signal(dist_dist)
    force_l, force_r =dist_pid.get_force_lateral(signal_dist, signal_phi) # Questa serve per il Kalman
    #kal.Fl = force_l
    #kal.Fr = force_r
    l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori
    #if l_pwm >= 0:
        # p2.ChangeDutyCycle(l_pwm)
    #else:
        #p2i.ChangeDutyCycle(-l_pwm)
    r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
    #if r_pwm >= 0:
        # p3.ChangeDutyCycle(r_pwm)
    #else:
        #p3i.ChangeDutyCycle(-r_pwm)

    if dist_dist < 0.1:
        val = val+1
        # aumenti un array trajectory
        z_pid.set_new_target(traj[val][0])
        phi_pid.set_new_target(traj[val][1])
        dist_pid.set_new_target(traj[val][2])


