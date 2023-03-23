# Main file for running the Blimp
# it uses all the functions present in "blimp class"

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers
#, kalman_blimp
from PID import PID_Controller
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


#SET OUTPUT MESSAGE
hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
rl = ReadLine(hser)

######################################################



###############################################################
# Codice per stabilire rotazione iniziale del Blimp
# nello spazio e stabilire orientazione dei suoi assi 
# rispetto al sistema di riferimento globale 
#################################################################


def blimp_to_world_rf(raw_acc, raw_gyr, raw_mag):
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
    mag_file = open("data_UWB2.txt","w")
    data_string = ""
    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    # Primo step, calcolo della orientazione iniziale del 
    # dirigibile nello spazio
    delta, ang_rad, quat_final, yaw_0 = blimp_to_world_rf(icm.acceleration,icm.gyro, icm.magnetic) #==> delta, ang rad
     # Plotting the final results
    print("Differenza Angolo Madgwick e assoluto = ", delta*180/np.pi)
    print("Angolo nel Global RF = ", ang_rad*180/np.pi)
    #print("quat_final = ", quat_final*180/np.pi)
    print("yaw_0  del blimp per il Madgwick= ", yaw_0*180/np.pi)


    
    ##############################################
    # UWB
    #############################################
    tempi_dt = np.zeros((1,6))



    try:
        mesg0 = rl.readline().decode("utf-8")
        #print(mesg)
        ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        #print(dt_uwb)
        tempi_dt[0,:] = dt_uwb





        # Qui posso lanciare tutte le operazioni preliminari
        # per il corretto funzionamento del codice

        # Primo step, calcolo della orientazione iniziale del 
        # dirigibile nello spazio
        #delta, ang_rad, quat_final = blimp_to_world_rf(icm.acceleration,icm.gyro, icm.magnetic) #==> delta, ang rad

        # Calcolare la traiettoria da seguire con il path planner
        '''
        #Madgwick initialization
        q0 = Quaternion(0, 0, 0, 1)
        mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1) 
        
        # PID initialization
        #dist_pid = PID_Controller(kp= 1, ki = 1, kd = 1, max_signal =12, sample_rate=1.0, target=0.0)
        psi_pid = PID_Controller(kp= 10, ki = 0.0000, kd = 0.0,max_signal =12, sample_rate=1.0, target=0.0)
        z_pid = PID_Controller(kp= 1, ki = 0, kd = 0, max_signal =12, sample_rate=1.0, target=0.0)

        traj = np.array([[0, 0, 1],
                        [1, 1, 1],
                        [2, 2, 2]])
        
        # 


        # Mantenimento di una data quota
        z_target = 0.3 # metri
        psi_target = np.pi/3.0 #rad
        #point_target = np.array([5.0, 2.0]) #array of coordinates x,y  m
        #dist_target = 0.01 #m distance the blimp want to minimize
        # Kalman initialization
        # kal = kalman_blimp()

        ####################################################
        # SET INITIAL TARGET FOR PID OBJECT CLASS
        z_pid.set_new_target(z_target)
        psi_pid.set_new_target(psi_target)
        #dist_pid.set_new_target(dist_target)
        #####################################################

        
        # Contatore per le misure fatte a 1 HZ per IMU e Sonar
        zero_measure_time = time.perf_counter()
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
            #print(quaternion)
            time_zero = time.perf_counter() #ORIGINAL TIME ZERO FOR MADGWICK ALONE
            quat = Quaternion(quaternion)
            roll, pitch, yaw = quat.to_euler123()
            psi = yaw # measured orientation
            
            #######################
            # UWB ALWAYS WORKING
            ########################
            mesg = rl.readline().decode("utf-8")
            #print(mesg)
            ts = mesg.split(" ")
                
            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            for i in range(len(dt_new)) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[i,:], m=2))
            #dt_uwb = np.mean(reject_outliers(tempi_dt))
            #dt_uwb = dt_new
            print("coordinate = ", [x_pos_uwb, y_pos_uwb, z_pos_uwb])

            
            
            
            # Other sensors
            current_measure_time = time.perf_counter()
            if (current_measure_time - zero_measure_time) >= 1.0:
                pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel = misuration()
                zero_measure_time = time.perf_counter()

                ##############################################
                # Z_PID block of code 
                ##############################################
                #z_pid.sample_rate = time_fine - time_zero al momento è costante
                signal_z = z_pid.adjust_signal(pos_z)
                force_z =z_pid.get_force_z(signal_z) # Questa serve per il Kalman
                #kal.Fu = force_z
                z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori
                print("Signal_Z =", signal_z)
            
                #if Npwm_z >= 0:
                    # p1.ChangeDutyCycle(Npwm_z)
                #else:
                    #p1i.ChangeDutyCycle(-Npwm_z)
            
                ###############################
                ###############################
                # dist and phi pid block of code
                # dist_init = 
                #dist_dist = math.sqrt((pos_x-point_target[0])**2 + (pos_y - point_target[1])**2)
                
                #phi_pid.sample_rate = time_fine - time_zero AL momento è costante
                #dist_pid.sample_rate = time_fine - time_zero
                #time_zero = time.perf_counter()
                signal_psi = psi_pid.adjust_signal(psi)
                print("Yaw angle =", psi*180/np.pi)
                print("Signal_Yaw =", signal_psi)
                #signal_dist = dist_pid.adjust_signal(dist_dist)
                #force_l, force_r =dist_pid.get_force_lateral(signal_dist, signal_phi) # Questa serve per il Kalman
                #kal.Fl = force_l
                #kal.Fr = force_r
                #l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori
                #print("Signal_dist =", signal_dist)
                #if l_pwm >= 0:
                    # p2.ChangeDutyCycle(l_pwm)
                #else:
                    #p2i.ChangeDutyCycle(-l_pwm)
                #r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                #print("Signal_phi =", signal_phi)
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
                
        '''

    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')




