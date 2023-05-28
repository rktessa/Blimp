# -*- coding: utf-8 -*-

# Main file for running the Blimp
# it uses all the functions present in "blimp class"
# ps aux | grep serial

# VERSIONE  09 MAGGIO 2023 - Kalman

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt 
from blimp_class import reject_outliers, PID_Controller, psi_map, Astar, reshape_z 
from blimp_class import kalman_blimp, imu_to_uwb, rotation_UWB
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

    log_file = open("data_Kalman.txt","w")
    data_string = ""
         
    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME

    yaw_0, quat_final, x_0, y_0, z_0 = blimp_to_world_rf()
    
    # Printing the final results
    print("yaw_0 del blimp per il Madgwick= ", yaw_0*180/np.pi) # deg
    
    # Goal position
    x_goal = 1
    y_goal = 5
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_0
    y_pos = y_0
    z_pos = z_0

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
        mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.3) 
        
        dist_target = 0.01 # m distance the blimp want to minimize
        z_target = 0.5  # Mantenimento di una data quota
        
        # PID initialization
        dist_pid = PID_Controller(kp= 0.01, ki = 0.001, kd = 0.0, max_signal =100000, sample_rate=1.0, target=dist_target)
        psi_pid = PID_Controller(kp= 0.01, ki = 0.001, kd = 0.0, max_signal =100000, sample_rate=1.0, target=0.0)
        z_pid = PID_Controller(kp= 0.1, ki = 0.01, kd = 0.0, max_signal =100000, sample_rate=1.0, target=z_target)

        # Contatore per le misure fatte a 2 HZ per UWB e Sonar
        time_zero_2Hz = time.perf_counter()
        # Contatore per le misure fatte a 1 HZ per PID
        time_zero_1Hz = time.perf_counter()
        # Zero time for Madgwick
        time_zero_mad = time.perf_counter()
        # Zero time for Kalman
        time_zero_kal = time.perf_counter()

        # Initialize the sonar for a first value
        z_sonar, flag = Sonar()
        
        if flag:
          z_pos = z_sonar

        #########################
        # KALMAN self, dt=None, Fl= None, Fr=None, Fu=None, x_pos=None, y_pos=None, z_pos=None, phi_pos=None
        kal = kalman_blimp(dt =0.05 , Fl = 0.013,  Fr = 0.013, Fu = 0, phi_pos=0, x_pos=x_pos, y_pos=y_pos, z_pos=0.5)
        # When the class is initialized, here I can insert the intial coordinate
        # for position and orientation in Global reference frame. 
        
        F, B, u_t ,x, P , Q, H, R = kal.initialization() 
        
        # Per far funzionare il codice anche se c'è un outliers nella prima misura
        x_pos_mes = x_pos
        y_pos_mes = y_pos         
        
        while 1: 

           
            ########################
            # UWB 
            ########################
            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
            
            if (len(ts)!=25):
              mesg = rl.readline().decode("utf-8")
              ts = mesg.split(" ")

                      
            
            # Sonar and UWB and Madgwick work at 2 HZ for refresh the PID and the motor values
            if (time.perf_counter() - time_zero_2Hz) >= 0.5:
                
                ##### KALMAN PREDICT E UPDATE ##############################
                x, P = kal.predict_kal(x=x, P=P, F=F, Q=Q, u=u_t, B=B)

                kal.dt = time.perf_counter() - time_zero_kal
                time_zero_kal = time.perf_counter() 

                # Madgwick orientation estimation (IMU) always working
            
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
                
                psi = psi_mapped - yaw_0*180/np.pi # deg
                if psi < 0:
                    psi = psi + 360

                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
                
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
                
                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x_pos_mes = x_pos_uwb
                    y_pos_mes = y_pos_uwb            
                         

                z_sonar, flag = Sonar()
                zero_measure_time2 = time.perf_counter()
                if flag:
                  z_pos = z_sonar
                
                # x_pos, y_pos, psi and z_pos are our measurements
                
                time_zero_2Hz = time.perf_counter()

                # Il vettore measure deve tenere conto di come è orientato l'IMU! 
                #  Il più 9.81 ha senso adesso ???????????????????????
                measure = np.array([x_pos_mes, accelerometer[1], y_pos_mes, accelerometer[0], 0.5, -accelerometer[2]+9.81, psi*np.pi/180, gyroscope[2]]).T
                    
                x, P = kal.update_kal(x, P, measure, R, H)
                # Uso i valori calcolati con il Kalman
                x_pos = x[0]
                y_pos = x[2]
                z_pos = x[4] 
                psi = x[6] 

                # Print measure and estimated values:
                print(x_pos_mes, y_pos_mes, x_pos, y_pos, psi, kal.Fl, kal.Fr, x[1])

                data_string = str(x_pos) + ", " + str(y_pos) + ", " + str(z_pos) + ", " + str(psi) + ", " + str(measure[0]) + ", " + str(measure[2]) + ", " + str(measure[4]) + ", " + str(measure[6]) + "\n"
                log_file.write(data_string)

            if (time.perf_counter() - time_zero_1Hz) >= 1.0:

                # distance from the target calculation
                goal_dist = math.sqrt((x_pos-x_goal)**2 + (y_pos - y_goal)**2)
                
                # psi target calculation
                psi_target = math.atan2((y_goal-y_pos), (x_goal-x_pos)) # rad
                psi_target_deg = psi_target*180/np.pi
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
                # kal.Fu = z_pwm * 0.1/100
                
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
                
                if abs(psi - psi_target) > 10:
                    force_l, force_r = dist_pid.get_force_lateral(0, signal_psi) # Questa serve per il Kalman
                else:
                    force_l, force_r = dist_pid.get_force_lateral(signal_dist, 0) # Questa serve per il Kalman
                    
                    
                # kal.Fl = force_l
                # kal.Fr = force_r
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
                
                #print("dist, psi e psi_target = ", dist_dist, psi*180/np.pi, psi_target*180/np.pi)
                #print("force_l, force_r, l_pwm e r_pwm = ", force_l, force_r, l_pwm, r_pwm)
                
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



