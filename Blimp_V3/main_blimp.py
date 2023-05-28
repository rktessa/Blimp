# -*- coding: utf-8 -*-

# Main file for running the Blimp
# it uses all the functions present in "blimp class"
# ps aux | grep serial

# VERSIONE  13 APRILE 2023 - No Kalman

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, PID_Controller, psi_map, Astar, psi_mean
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

# Classe per UWB
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


'''
# Info per i messaggi
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://192.168.1.176:5556")
'''


# Definizione di tutti i pin per uso
m1 = 17 # sinistro
m1_i = 27
m2 = 13  # destro
m2_i = 19
m3 =  20 # sotto
m3_i =  21

HCtrig=8     
HCecho=7     

IO.setwarnings(False)           
IO.setmode(IO.BCM)        # numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori  
IO.setup(m1_i,IO.OUT)
IO.setup(m2,IO.OUT)
IO.setup(m2_i,IO.OUT)
IO.setup(m3,IO.OUT)
IO.setup(m3_i,IO.OUT)  
IO.setup(HCtrig,IO.OUT)   # setup ultrasuoni
IO.setup(HCecho,IO.IN)   

p1 = IO.PWM(m1,100)       # inizializzazione pwm        
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


'''
#################################################################
# Funzione per allineare i sistemi di riferimento del blimp e dell' UWB
#################################################################


def blimp_to_world_rf():

    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.3)
    yaw_vec = []
    print("Allinea il blimp con il sistema di riferimento UWB per 20 secondi")
        
    time_zero = time.perf_counter()
    time_start = time.perf_counter()
    while  time.perf_counter() < (time_start + 2 ):
            raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
            # Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)
            # Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
          
            time_fine = time.perf_counter()
            # setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
            time_zero = time.perf_counter()
            quat = Quaternion(quaternion)
            
            roll, pitch, yaw = quat.to_euler123()  # Result is in rad
            yaw_deg = yaw*180/np.pi
            if yaw_deg < 0:
              yaw_deg = yaw_deg + 360
            yaw_mapped = psi_map(yaw_deg)
            yaw_vec.append(yaw_mapped)
            
    
    quat_final = quat # the last quaternion is stored as input for the madgwick later
    psi_0 = sum(yaw_vec[-10 :])/10
    
    return psi_0, quat_final
    # quat_final is the initial input for Madgwick relaunch
    
    # Per usare la funzione blimp to world correttamente 
    # psi_mad_abs =  psi_mad_relative - psi_0. # Se mantengo gli angoli anche negativi funziona bene
    '''

def blimp_to_world_rf():
    
    # q0 = Quaternion(0, 0, 0, 1)
    q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.5, quaternion=q0, beta=0.3)
    psi_list = []
    x_list = []
    y_list = []
    z_list = []

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        mesg0 = rl.readline().decode("utf-8")
        ts = mesg0.split(" ")
        if (len(ts)!=25):
          mesg0 = rl.readline().decode("utf-8")
          ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        tempi_dt[0,:] = dt_uwb

        time_zero_mad = time.perf_counter()
        time_zero_meas = time.perf_counter()
        time_zero_UWB = time.perf_counter()

        print("Allinea il blimp con il sistema di riferimento assoluto per 20 secondi")
        while  time.perf_counter() < (time_zero_meas + 20):

            # Misuro con UWB la posizione nel piano  frattempo 
            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
            if (len(ts)!=25):
                mesg = rl.readline().decode("utf-8")
                ts = mesg.split(" ")

            time_current_UWB = time.perf_counter()
            if (time_current_UWB - time_zero_UWB) >= 0.5:

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
                
                psi = yaw*180/np.pi
                if psi < 0:
                    psi = psi + 360
                psi_mapped = psi_map(psi)
                psi_list.append(psi_mapped) # deg

                if len(psi_list) > 5:
                    psi_list.pop(0)

                psi_mean = sum(psi_list)/len(psi_list)
                
                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)

                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x_list.append(x_pos_uwb)
                    y_list.append(y_pos_uwb)
                    z_list.append(z_pos_uwb)
                    print(x_pos_uwb,y_pos_uwb,z_pos_uwb,psi_mean)

                if len(x_list) > 10:
                    x_list.pop(0)

                if len(y_list) > 10:
                    y_list.pop(0)

                if len(z_list) > 10:
                    z_list.pop(0)
              
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=2))

                time_zero_UWB = time.perf_counter()
    
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    psi_0 = sum(psi_list)/len(psi_list) # deg

    x_0 = sum(x_list)/len(x_list) # perform the mean of the last 20 element
    y_0 = sum(y_list)/len(y_list)
    z_0 = sum(z_list)/len(z_list)
    return psi_0, quat_final, x_0, y_0, z_0 # These are the values of initial angles
    # quat_final is the initial input for Madgwick relaunch
    

def Sonar():
    # From Sonar
    # Set Trigger to High
    IO.output(HCtrig, True)
    # Set trigger after 0.01 ms to Low
    time.sleep(0.00001)
    IO.output(HCtrig, False)

    StartTime = time.time()
    StopTime = time.time()
    
    counter = 1
    flag = 0

    while IO.input(HCecho)==0 and counter < 200:
        StartTime = time.time()
        counter = counter + 1
    while IO.input(HCecho)==1:
        StopTime = time.time()
    
    if counter < 200:
      pulse_duration = StopTime - StartTime
      distance = pulse_duration * 171.50 #result in meter
      flag = 1
    else:
      distance = 1
      flag = 0
    
    return distance, flag


def main():

    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME

    psi_0, quat_final, x_0, y_0, z_0 = blimp_to_world_rf()
    
    # Printing the final results
    print("psi_0 del blimp per il Madgwick= ", psi_0) # deg
    
    # Goal position
    x_goal = 2.5
    y_goal = 5
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_0
    y_pos = y_0
    z_pos = z_0
    
    # creo liste per posizioni x, y e z
    x_list = [x_pos]
    y_list = [y_pos]
    z_list = []
    psi_list = []

    #############################################
    # UWB
    #############################################
    tempi_dt = np.zeros((1,6))

    try:
        mesg0 = rl.readline().decode("utf-8")
        
        ts = mesg0.split(" ")
        
        if (len(ts)!=25):
          mesg0 = rl.readline().decode("utf-8")
          ts = mesg0.split(" ")
        
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        
        tempi_dt[0,:] = dt_uwb

        # Madgwick initialization
        q0 = quat_final
        #q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
        mad = Madgwick(sampleperiod = 0.5, quaternion=q0, beta=0.3) 
        
        dist_target = 0.01 # m distance the blimp want to minimize
        z_target = 0.5  # Mantenimento di una data quota
        
        # PID initialization
        dist_pid = PID_Controller(kp= 0.1, ki = 0.001, kd = 0.01, max_signal =100000, sample_rate=1.0, target=dist_target)
        psi_pid = PID_Controller(kp= 0.01, ki = 0.001, kd = 0.01, max_signal =100000, sample_rate=1.0, target=0.0)
        z_pid = PID_Controller(kp= 0.1, ki = 0.01, kd = 0.01, max_signal =100000, sample_rate=1.0, target=z_target)

        # Contatore per le misure fatte a 2 HZ per UWB, Sonar e Madgwick
        time_zero_2Hz = time.perf_counter()
        # Contatore per le misure fatte a 1 HZ per PID
        time_zero_1Hz = time.perf_counter()
        # Zero time for Madgwick
        time_zero_mad = time.perf_counter()

        # Initialize the sonar for a first value
        z_sonar, flag = Sonar()
        
        if flag:
          z_pos = z_sonar     
        
        while 1: 

            ########################
            # UWB 
            ########################
            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
            
            if (len(ts)!=25):
              mesg = rl.readline().decode("utf-8")
              ts = mesg.split(" ")

            # Sonar and UWB and Madgwick work at 2 Hz
            if (time.perf_counter() - time_zero_2Hz) >= 0.5:
                           
                #Aquisizione magnetometro e calibrazione dei dati:
                mag = calibration(icm.magnetic)
                # Creazione vettore input per classe Madgwick
                accelerometer, gyroscope, magnetometer = np.asarray(icm.acceleration), np.asarray(icm.gyro), np.asarray(mag)
                # setting the variable frequency of update of Madgwick alghorithm
                mad.samplePeriod = time.perf_counter() - time_zero_mad
                quaternion = mad.update(gyroscope, accelerometer, magnetometer)
                time_zero_mad = time.perf_counter()

                quat = Quaternion(quaternion)
                roll, pitch, yaw = quat.to_euler123()
                yaw_deg = yaw*180/np.pi # deg
                if yaw_deg < 0:
                    yaw_deg = yaw_deg + 360
                psi_mapped = psi_map(yaw_deg) # deg
                
                psi_IMU = psi_mapped - psi_0 # deg
                if psi_IMU < 0:
                    psi_IMU = psi_IMU + 360
                    
                psi_meas = 360 - psi_IMU

                psi_list.append(psi_meas)

                if len(psi_list) > 5:
                    psi_list.pop(0)

                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
                
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
                
                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x_pos = x_pos_uwb
                    y_pos = y_pos_uwb            
                         
                z_sonar, flag = Sonar()
                
                if flag:
                  z_pos = z_sonar
                
                # x_pos, y_pos, psi and z_pos are our measurements
                
                # last 5 x, y, z and psi measurements
                x_list.append(x_pos)
                y_list.append(y_pos)
                z_list.append(z_pos)
                if len(x_list) > 5:
                    x_list.pop(0)
                    y_list.pop(0)
                    z_list.pop(0)   
                
                time_zero_2Hz = time.perf_counter()

            if (time.perf_counter() - time_zero_1Hz) >= 1.0:

                x_pos = sum(x_list) / len(x_list)
                y_pos = sum(y_list) / len(y_list)
                z_pos = sum(z_list) / len(z_list)
                psi = psi_mean(psi_list,psi_meas)

                # print("State = ", x_pos, y_pos, z_pos, psi)

                # distance from the target calculation
                goal_dist = math.sqrt((x_pos-x_goal)**2 + (y_pos - y_goal)**2)
                
                # psi target calculation
                psi_target = (math.atan2((y_goal-y_pos), (x_goal-x_pos)))*180/np.pi # deg
                if psi_target < 0:
                    psi_target = psi_target + 360
                # Questo blocco serve nel caso lo psi target sia vicino allo 0
                psi_target_new = psi_target
                if 0 < psi_target and psi_target <= 90 and 270 < psi and psi <= 360:
                    psi_target_new = psi_target + 360
                if 270 < psi_target and psi_target <= 360 and 0 < psi and psi <= 90:
                    psi_target_new = psi_target - 360
                psi_target = psi_target_new # deg
                
                ####################################################
                # SET INITIAL TARGET FOR PID OBJECT CLASS
                z_pid.set_new_target(z_target)
                psi_pid.set_new_target(psi_target)
                dist_pid.set_new_target(dist_target)
                #####################################################

                ##############################################
                # Z_PID block of code 
                ##############################################

                signal_z = z_pid.adjust_signal(z_pos)
                force_z = z_pid.get_force_z(signal_z) # Questa serve per il Kalman
                z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori
                                
                #print("z_pos, signal_z e z_pwm = ", z_pos, signal_z, z_pwm)
            
                '''
                if z_pwm >= 0:
                    p1.ChangeDutyCycle(z_pwm)
                else:
                    p1i.ChangeDutyCycle(-z_pwm)
                '''
            
                # Per variare il sample rate del PID controller durante la run
                signal_psi = psi_pid.adjust_signal(psi)
                signal_dist = dist_pid.adjust_signal(-goal_dist)
                
                if goal_dist > 0.25:
                    if abs(psi - psi_target) > 15:
                        force_l, force_r = dist_pid.get_force_lateral(0, signal_psi) # Questa serve per il Kalman
                    else:
                        force_l, force_r = dist_pid.get_force_lateral(signal_dist, 0) # Questa serve per il Kalman
                        
                        
                    l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori
                    
                    '''
                    if l_pwm >= 0:
                        p2.ChangeDutyCycle(l_pwm)
                    else:
                        p2i.ChangeDutyCycle(-l_pwm)
                    '''

                    r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                    
                    
                    '''
                    if r_pwm >= 0:
                        p3.ChangeDutyCycle(r_pwm)
                    else:
                        p3i.ChangeDutyCycle(-r_pwm)
                    '''
                else:
                    print("Target reached: motors off")
                    l_pwm = 0
                    r_pwm = 0
                    '''
                    p2.ChangeDutyCycle(l_pwm)
                    p2i.ChangeDutyCycle(-l_pwm)
                    p3.ChangeDutyCycle(r_pwm)
                    p3i.ChangeDutyCycle(-r_pwm)
                    '''
                
                #print("dist, psi e psi_target = ", dist_dist, psi*180/np.pi, psi_target*180/np.pi)
                print("psi, psi_target e goal distance = ", psi, psi_target, goal_dist)
                print("l_pwm, r_pwm e z_pwm = ", l_pwm, r_pwm, z_pwm)
                
                '''
                # Invio della posizione per la stampa
                zipcode = 10001
                mes_x = x_pos
                mes_y = y_pos
                #socket.send_string(f"{zipcode} {mes_x} {mes_y}")
                '''

                time_zero_1Hz = time.perf_counter()
            
                
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')





if __name__ == "__main__":

    # Codice per provare a usare UWB
    mesg = {}
    mesg0 = {}
    # SET OUTPUT MESSAGE
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)

    try:    
        main()  

    except TypeError: 
        print("Crashed :/")
        hser.close()
        print ('Serial port closed')
        p1.ChangeDutyCycle(0)
        p1i.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p2i.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        p3i.ChangeDutyCycle(0)
        print("Motors stopped") 
        logging.exception("message")

    except ValueError: 
        print("Crashed :/")
        hser.close()
        print ('Serial port closed')
        p1.ChangeDutyCycle(0)
        p1i.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p2i.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        p3i.ChangeDutyCycle(0)
        print("Motors stopped") 
        logging.exception("message")



    except KeyboardInterrupt:
        print("Interrupted")
        hser.close()
        print ('Serial port closed')
        p1.ChangeDutyCycle(0)
        p1i.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p2i.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        p3i.ChangeDutyCycle(0)
        print("Motors stopped")



