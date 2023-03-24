# Main file for running the Blimp
# it uses all the functions present in "blimp class"
# ps aux | grep serial


import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map, Astar
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
import cv2
import logging


# Per salvare i risultati dell'UWB in un file di testo
# mag_file = open("data_UWB.txt","w")
# data_string = ""

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



###############################################################
# Codice per stabilire rotazione iniziale del Blimp
# nello spazio e stabilire orientazione dei suoi assi 
# rispetto al sistema di riferimento globale 
#################################################################


def blimp_to_world_rf():

    ##################################################################
    # Codice per provare a usare UWB
    mesg_0 = {}

    # SET OUTPUT MESSAGE
    hser0 = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl0 = ReadLine(hser0)

    ######################################################


    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []
    x = np.empty((1,0))
    y = np.empty((1,0))

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        mesg0 = rl0.readline().decode("utf-8")
        ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        tempi_dt[0,:] = dt_uwb

        time_zero = time.perf_counter()
        time_start = time.perf_counter()
        while  time.perf_counter() < (time_start + 20 ):
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
            
            yaw_mapped = psi_map(yaw*180/np.pi)
            roll_vec.append(roll)
            pitch_vec.append(pitch)
            yaw_vec.append(yaw_mapped*np.pi/180)
            
            # Misuro con UWB la posizione nel piano  frattempo 
            mesg_0 = rl0.readline().decode("utf-8")
            
            ts = mesg_0.split(" ")
            
            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)

            if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                x = np.append(x, x_pos_uwb)
                y = np.append(y, y_pos_uwb)
                print(x_pos_uwb,y_pos_uwb)
            
            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            
            for i in range(len(dt_new)) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[i,:], m=2))

            # Save the UWB results in a .txt file
            # data_string = str(x_pos_uwb) + " ," + str(y_pos_uwb) + " ," + str(z_pos_uwb) + " ," + str(yaw) + "\n"
            # mag_file.write(data_string)
            
    
    except KeyboardInterrupt:
        hser0.close()
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    roll_0 = sum(roll_vec[-40 :])/40 # perform the mean of the last 40 element
    pitch_0 = sum(pitch_vec[-40 :])/40
    yaw_0 = sum(yaw_vec[-40 :])/40
    
    # Ora usando i dati creati vado a fare una LLS estimation
    # y = a + b*x
    a = (np.sum(y) * np.sum(x**2)- np.sum(x)*np.sum(x*y))/ (len(x)*np.sum(x**2) - np.sum(x)**2)
    b = ( len(x)* np.sum(x*y) - np.sum(x)*np.sum(y)) /(len(x)*np.sum(x**2)- np.sum(x)**2) 

    # The error of my model
    
    ang_rad = np.arctan(b) # the inclination of the line calculated in rad with uwb
    delta = yaw_0 - ang_rad

    # Prima di finire chiudi la serial port
    hser0.close()

    return delta, ang_rad, quat_final, yaw_0, x[-1], y[-1] # These are the values of initial angles
    # This ang_rad is the parameter to insert in both the A* and PID phi as initial orientation calculation
    # quat_final is the initial input for Madgwick relaunch
    
    # Per usare la funzione blimp to world correttamente 
    # psi_mad_abs =  psi_mad_relative - delta. # Se mantengo gli angoli anche negativi funziona bene

# Function to call for get measurement of all the sensors togheter
def Sonar():
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
    z_sonar = round(distance, 3)
    
    return z_sonar


# MANCA IL KALMAN E L'A*



def main():
         
    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    # Primo step, calcolo della orientazione iniziale del 
    # dirigibile nello spazio
    delta, ang_rad, quat_final, yaw_0, x_pos_init, y_pos_init = blimp_to_world_rf() #==> delta, ang rad
    # Plotting the final results
    print("Differenza Angolo Madgwick e assoluto = ", delta*180/np.pi)
    print("Angolo nel Global RF = ", ang_rad*180/np.pi)
    # print("quat_final = ", quat_final)
    print("yaw_0  del blimp per il Madgwick= ", yaw_0*180/np.pi)

    #############################################################################################################
    # Path Planning prova
    
    print("Inizio path planning")
    img = cv2.flip(cv2.imread("/home/piblimp/Blimp_23/Blimp_V2/lab_meccatronica.png"),0)
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20, 20)))
    img = img.astype(float)/255.
    #############################################################################################################


    start = (int(x_pos_init*100),int(y_pos_init*100)) ## MY case
    goal = (255, 760)

    
    astar = Astar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=10) #10 è 1 metro nella realtà
    traj = np.array(path)/100.0
    print(traj) # ==> questo è il percorso da seguire
    



    # indice punto della traiettoria
    val = 0
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_pos_init
    y_pos = y_pos_init

    ##############################################
    # UWB
    #############################################
    tempi_dt = np.zeros((1,6))

    # Codice per provare a usare UWB
    mesg = {}

    # SET OUTPUT MESSAGE
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)



    try:
        mesg0 = rl.readline().decode("utf-8")
        
        ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        
        tempi_dt[0,:] = dt_uwb

        # Qui posso lanciare tutte le operazioni preliminari
        # per il corretto funzionamento del codice

        # Calcolare la traiettoria da seguire con il path planner

        # Madgwick initialization
        q0 = quat_final
        mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35) 
        
        # PID initialization
        dist_pid = PID_Controller(kp= 1, ki = 1, kd = 1, max_signal =12, sample_rate=1.0, target=0.0)
        psi_pid = PID_Controller(kp= 10, ki = 0.0000, kd = 0.0,max_signal =12, sample_rate=1.0, target=0.0)
        z_pid = PID_Controller(kp= 1, ki = 0, kd = 0, max_signal =12, sample_rate=1.0, target=0.0)


        # Kalman initialization
        # kal = kalman_blimp()

        # Contatore per le misure fatte a 1 HZ per IMU e Sonar
        zero_measure_time = time.perf_counter()
        # SET ZERO TIME, FUNDAMENTAL BEFORE THE WHILE LOOP
        time_zero = time.perf_counter()

        while 1: 
            
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(icm.magnetic)
            # Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(icm.acceleration), np.asarray(icm.gyro), np.asarray(mag)
            time_fine = time.perf_counter()
            # setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
           
            time_zero = time.perf_counter() #ORIGINAL TIME ZERO FOR MADGWICK ALONE
            quat = Quaternion(quaternion)
            roll, pitch, yaw = quat.to_euler123()
            psi_mapped = psi_map(yaw*180/np.pi) - delta*180/np.pi # deg
            
            psi = psi_mapped * np.pi/180

            #######################
            # UWB ALWAYS WORKING
            ########################
            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
                
            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)


            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            for i in range(len(dt_new)) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[i,:], m=2))
            
            if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                x_pos = x_pos_uwb
                y_pos = y_pos_uwb
                # print("coordinate = ", [x_pos_uwb, y_pos_uwb, z_pos_uwb])

            ###############################
            ###############################
            # dist and phi pid block of code
            dist_dist = math.sqrt((x_pos-traj[val][0])**2 + (y_pos - traj[val][1])**2)
            
            # Other sensors, they cycle at 1 HZ for refresh the PID and the motor values
            current_measure_time = time.perf_counter()
            if (current_measure_time - zero_measure_time) >= 1.0:
                
                
                z_pos = Sonar()
                # x_pos, y_pos, psi and z_pos are our measurements

                # Manipolazione della psi per gestire la navigazione con un range di misure che va da 0 a 2 pi
                psi_target = math.atan2((traj[val][1]-y_pos), (traj[val][0]-x_pos)) # rad
                if psi_target < 0:
                    psi_target = psi_target + 2*np.pi
                if 0 < psi_target and psi_target <= np.pi/2 and 3/2*np.pi < psi and psi <= 2*np.pi:
                    psi = psi - 2*np.pi
                if 3/2*np.pi < psi_target and psi_target <= 2*np.pi and 0 < psi and psi <= np.pi/2:
                    psi = psi + 2*np.pi

               
                z_target = 120  # Mantenimento di una data quota

                dist_target = 0.01 # m distance the blimp want to minimize

                ####################################################
                # SET INITIAL TARGET FOR PID OBJECT CLASS
                z_pid.set_new_target(z_target)
                psi_pid.set_new_target(psi_target)
                dist_pid.set_new_target(dist_target)
                #####################################################

                zero_measure_time = time.perf_counter()

                ##############################################
                # Z_PID block of code 
                ##############################################

                # z_pid.sample_rate = time_fine - time_zero # al momento è costante
                signal_z = z_pid.adjust_signal(z_pos)
                force_z = z_pid.get_force_z(signal_z) # Questa serve per il Kalman
                # kal.Fu = force_z
                z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori
                            
                '''if z_pwm >= 0:
                    p1.ChangeDutyCycle(z_pwm)
                else:
                    p1i.ChangeDutyCycle(-z_pwm)'''
            
                # Per variare il sample rate del PID controller durante la run
                # phi_pid.sample_rate = time_fine - time_zero # al momento è costante
                # dist_pid.sample_rate = time_fine - time_zero
                # time_zero = time.perf_counter()

                signal_psi = psi_pid.adjust_signal(psi)
                
                signal_dist = dist_pid.adjust_signal(dist_dist)
                force_l, force_r = dist_pid.get_force_lateral(signal_dist, signal_psi) # Questa serve per il Kalman
                # kal.Fl = force_l
                # kal.Fr = force_r
                l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori
                
                '''if l_pwm >= 0:
                    p2.ChangeDutyCycle(l_pwm)
                else:
                    p2i.ChangeDutyCycle(-l_pwm)'''

                r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                
                print( "PWM =", l_pwm, "   ", r_pwm )
                '''if r_pwm >= 0:
                    p3.ChangeDutyCycle(r_pwm)
                else:
                    p3i.ChangeDutyCycle(-r_pwm)'''
                
            if dist_dist < 0.1:
                    val = val + 1
                    # aumenti un array trajectory
                    z_pid.set_new_target(z_target)
                    psi_target = math.atan2((traj[val][1]-y_pos), (traj[val][0]-x_pos)) # rad
                    if psi_target < 0:
                        psi_target = psi_target + 2*np.pi
                    psi_pid.set_new_target(psi_target)
                    dist_pid.set_new_target(dist_target)
                
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')





if __name__ == "__main__":
    try:
        main()  

    except TypeError: 
        print("Crashed :/")
        hser0.close()
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
        hser0.close()
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
        hser0.close()
        hser.close()
        print ('Serial port closed')
        p1.ChangeDutyCycle(0)
        p1i.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p2i.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        p3i.ChangeDutyCycle(0)
        print("Motors stopped")



