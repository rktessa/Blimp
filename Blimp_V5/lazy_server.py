#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt 
from blimp_class import reject_outliers, PID_Controller, psi_map, Astar, reshape_z 
from blimp_class import kalman_blimp, imu_to_uwb, rotation_UWB
import numpy as np
import math
from numpy.linalg import norm, inv
from quaternion import Quaternion


############# FUNZIONI
def blimp_to_world_rf(time_zero, position, acc, gyro, mag):
    
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
    
    yaw_vec = []
    x = np.empty((1,0))
    y = np.empty((1,0))
    

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
       

        
        
        
        time_zero_meas = time.perf_counter()
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
                yaw_mapped = psi_map(psi)
                #roll_vec.append(roll)
                #pitch_vec.append(pitch)
                yaw_vec.append(yaw_mapped*np.pi/180)

                
                
                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)

                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x = np.append(x, x_pos_uwb)
                    y = np.append(y, y_pos_uwb)
                    #z = np.append(z, z_pos_uwb)
                    print(x_pos_uwb,y_pos_uwb,z_pos_uwb)
              
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=2))

                time_zero_UWB = time.perf_counter()
    
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    #roll_0 = sum(roll_vec[-20 :])/20 # perform the mean of the last 20 element
    #pitch_0 = sum(pitch_vec[-20 :])/20
    yaw_0 = sum(yaw_vec[-20 :])/20 # rad

    x_0 = sum(x[-20 :])/20 # perform the mean of the last 20 element
    y_0 = sum(y[-20 :])/20
    #z_0 = sum(z[-20 :])/20
    z_0 = z_pos_uwb
    return yaw_0, quat_final, x_0, y_0, z_0 # These are the values of initial angles
    # quat_final is the initial input for Madgwick relaunch



if __name__ == "__main__":

    # Definizione context per zmq
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://192.168.1.147:5556")
     # 1. Madgwick

     # 2. Kalman


    '''
    Ricevi gli aggiornamenti e fai i calcoli il più velocemente possibile. 
    Poi aggiorna le forze una volta al secondo nel modello del kalman
    e anche veramente nel modello del blimp. 
    Salva uno storico di log delle info dei sensori e dello stato. 
    '''
    while True:
        #  Wait for next request from client
        message = socket.recv_string()
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ = message.split(" ")
        print(f"Received Infos for the execution of the navigation")

        #  Do some 'work'
       
        

        #time.sleep(1)
        g = float(raw_accZ)
        #  Send reply back to client
        socket.send_string(f"Accelerazione di gravity %f" % g)