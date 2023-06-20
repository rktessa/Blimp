#
#   Per funzionare devi importare ache il file global e il file submodule
#   In submodule devi inserire indirizzo ip del blimp 192.168.1.104
# global_.pwm_left = variabile
# test_func()

import time
import zmq
from blimp_class import *
import numpy as np
import math
from numpy.linalg import norm, inv
from quaternion import Quaternion
from sub_module import *
import global_
import logging


try:
    # Definizione context per zmq
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.1.104:5556")
    print("Mi sono connesso al Raspberry pi Blimp")

    # Subscribe to zipcode, default is Blimp
    zip_code = "Blimp"
    socket.setsockopt_string(zmq.SUBSCRIBE, zip_code)
except KeyboardInterrupt:
    print("Interrupted")
    logging.exception("message")
    

def str_to_float():
    try:
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
    except KeyboardInterrupt:
        print("Interrupted")
        logging.exception("message")



def blimp_to_world_rf():
    
    # q0 = Quaternion(0, 0, 0, 1)
    q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.5, quaternion=q0, beta=0.1)
    psi_list = []
    x_list = []
    y_list = []
    z_list = []

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
        ts = mesg0.split(" ")
        if (len(ts)!=25):
          tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
          ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        tempi_dt[0,:] = dt_uwb

        time_zero_mad = tempo
        time_zero_meas = time.perf_counter()
        time_zero_UWB = time.perf_counter()

        print("Allinea il blimp con il sistema di riferimento assoluto per 20 secondi") 
        while  time.perf_counter() < (time_zero_meas + 5):

            # Misuro con UWB la posizione nel piano  frattempo 
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
            ts = mesg.split(" ")
            if (len(ts)!=25):
                tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
                ts = mesg.split(" ")

            time_current_UWB = time.perf_counter()
            if (time_current_UWB - time_zero_UWB) >= 0.09:

                raw_acc = [raw_accX, raw_accY, raw_accZ]
                raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
                raw_mag = [raw_magX, raw_magY, raw_magZ]
                
                
                # Aquisizione magnetometro e calibrazione dei dati:
                mag = calibration(raw_mag)

                raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag)

                # Creazione vettore input per classe madgwick
                accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
            
                # setting the variable frequency of update of Madgwick alghorithm
                mad.samplePeriod = tempo - time_zero_mad
                quaternion = mad.update(gyroscope, accelerometer, magnetometer)
                time_zero_mad = tempo
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
                    z_list.append(zpos)
                    print(x_pos_uwb,y_pos_uwb,zpos,psi,psi_mean)

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
        
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    psi_0 = sum(psi_list)/len(psi_list) # deg

    x_0 = sum(x_list)/len(x_list) # perform the mean of the last 20 element
    y_0 = sum(y_list)/len(y_list)
    z_0 = sum(z_list)/len(z_list)
    return psi_0, quat_final, x_0, y_0, z_0 # These are the values of initial angles
    # quat_final is the initial input for Madgwick relaunch


def main():

    # ORIENTAZIONE NEL GLOBAL REFERENCE FRAME

    psi_0, quat_final, x_0, y_0, z_0 = blimp_to_world_rf()
    
    # Printing the final results
    print("psi_0 del blimp per il Madgwick= ", psi_0) # deg
    
    
    # Goal position
    x_goal = x_0 + 1.5
    y_goal = y_0 
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_0
    y_pos = y_0
    z_pos = z_0
    
    # creo liste per posizioni x, y e z
    x_list = [x_pos]
    y_list = [y_pos]
    z_list = []
    psi_list = []

     # per UWB
    tempi_dt = np.zeros((1,6))
    # Ricezione dati da ZeroMQ
    tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()

    ts = mesg0.split(" ")
    if (len(ts)!=25):
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
        ts = mesg0.split(" ")
    dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
    tempi_dt[0,:] = dt_uwb

    # Madgwick initialization
    q0 = quat_final
    #q0 = Quaternion(0.7068252, 0, 0, 0.7073883)
    mad = Madgwick(sampleperiod = 0.5, quaternion=q0, beta=0.1) 
    
    dist_target = 0.01 # m distance the blimp want to minimize
    z_target = 0.5  # Mantenimento di una data quota
    
    # PID initialization
    dist_pid = PID_Controller(kp= 0.1, ki = 0.001, kd = 0.01, max_signal =100000, sample_rate=1.0, target=dist_target)
    psi_pid = PID_Controller(kp= 0.01, ki = 0.001, kd = 0.001, max_signal =100000, sample_rate=1.0, target=0.0)
    z_pid = PID_Controller(kp= 0.1, ki = 0.01, kd = 0.01, max_signal =100000, sample_rate=1.0, target=z_target)

    #########################
    # KALMAN self, dt=None, Fl= None, Fr=None, Fu=None, x_pos=None, y_pos=None, z_pos=None, phi_pos=None
    kal = kalman_blimp(dt =0.05 , Fl = 0.0,  Fr = 0.0, Fu = 0, phi_pos=0, x_pos=x_pos, y_pos=y_pos, z_pos=0.5)
    # When the class is initialized, here I can insert the intial coordinate
    # for position and orientation in Global reference frame. 
    
    F, B, u_t ,x, P , Q, H, R = kal.initialization() 



    accel = []

    #Contatori
    time_zero_mad = tempo
    
    time_zero_UWB = time.perf_counter()
    time_zero_1Hz = time.perf_counter()
    while True: 
        # Misuro con UWB la posizione nel piano  frattempo 
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
        ts = mesg.split(" ")
        if (len(ts)!=25):
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
            ts = mesg.split(" ")   

        time_current_UWB = time.perf_counter()
        #if (time_current_UWB - time_zero_UWB) >= 0.067:

        raw_acc = [raw_accX, raw_accY, raw_accZ]
        raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
        raw_mag = [raw_magX, raw_magY, raw_magZ]
        
        
        # Aquisizione magnetometro e calibrazione dei dati:
        # hard and soft iron calibration of magnetometer
        mag = calibration(raw_mag)
        # rotate the IMU reference frame with the UWB reference fram
        raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag) 

        # Creazione vettore input per classe madgwick
        accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
        
        # setting the variable frequency of update of Madgwick alghorithm
        mad.samplePeriod = tempo - time_zero_mad
        quaternion = mad.update(gyroscope, accelerometer, magnetometer)
        
        quat = Quaternion(quaternion)
        

        ##### KALMAN PREDICT E UPDATE ##############################
        x, P = kal.predict_kal(x=x, P=P, F=F, Q=Q, u=u_t, B=B)

        kal.dt = tempo - time_zero_mad #time.perf_counter() - time_zero_kal
        
        time_zero_mad = tempo


       


        roll, pitch, yaw = quat.to_euler123()  # Result is in rad
        # Qua ruoto le accelerazioni per portarle correttamente nel modello del kalman filter
        angle_1 = [roll*180/np.pi, pitch*180/np.pi, yaw*180/np.pi]
        
        acc_UWB, gyro_UWB = rotation_UWB(raw_acc, raw_gyr, angle_1)
        #print((acc_UWB[0,0],acc_UWB[0,1],acc_UWB[0,2]))
        accel.append((acc_UWB[0,0],acc_UWB[0,1],acc_UWB[0,2]))
        #print(accel)
        accel_mean = np.mean(accel, axis=0)
        acc_abs = acc_UWB - accel_mean#- np.array([[0, 0, 10.07]])
        # acc_abs is the argument for the Kalman
        print(acc_abs)

        yaw_deg = yaw*180/np.pi # deg
        
        # Put the yaw into 0-360 degree range and then map it
        if yaw_deg < 0:
            yaw_deg = yaw_deg + 360
        psi_mapped = psi_map(yaw_deg) # deg
        
        # Remove the offset to allign with the UWB Reference frame (the room)
        psi = psi_mapped - psi_0 # deg
        if psi < 0: 
            psi = psi + 360
            
        
        # Mean of the psi value to estimate the yaw more accurately
        psi_list.append(psi)

        if len(psi_list) > 3:
            psi_list.pop(0)

        x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
        
        dt_new = np.reshape(dt_new, (1,6))
        tempi_dt = np.append(tempi_dt,dt_new, axis=0)
        for i in range(6) :
            dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
        
        if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
            x_pos = x_pos_uwb
            y_pos = y_pos_uwb 

        # last 5 x, y, z and psi measurements again a mean is performed to increase the accuracy of the measure
        x_list.append(x_pos)
        y_list.append(y_pos)
        z_list.append(zpos)
        if len(x_list) > 5:
            x_list.pop(0)
            y_list.pop(0)
            z_list.pop(0)   


        # Scrivere i dati  
        stringa_imu = [tempo, raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2], mag[0], mag[1], mag[2], x_pos, y_pos, z_pos, psi ]
        res = " ".join([str(j) for j in stringa_imu])
        f.write(res)
        f.write("\n")


        time_zero_UWB = time.perf_counter()
        
        x_pos = sum(x_list) / len(x_list)
        y_pos = sum(y_list) / len(y_list)
        z_pos = sum(z_list) / len(z_list)
        psi = psi_mean(psi_list,psi)

         # UPDATE
        measure = np.array([x_pos, acc_abs[0], y_pos, acc_abs[1], z_pos, acc_abs[2], psi*np.pi/180, gyro_UWB[2]]).T
            
        x, P = kal.update_kal(x, P, measure, R, H)
        # Uso i valori calcolati con il Kalman
        x_pos = x[0]
        y_pos = x[2]
        z_pos = x[4] 
        psi = x[6] 

        # Print measure and estimated values:
        print(x_pos_uwb, y_pos_uwb, x_pos, y_pos, psi, kal.Fl, kal.Fr, x[1])

        data_string = str(x_pos) + ", " + str(y_pos) + ", " + str(z_pos) + ", " + str(psi) + ", " + str(measure[0]) + ", " + str(measure[2]) + ", " + str(measure[4]) + ", " + str(measure[6]) + "\n"
        l.write(data_string)
        l.write("\n")


        if (time.perf_counter() - time_zero_1Hz) >= 1.0:

            

            # print("State = ", x_pos, y_pos, z_pos, psi)

            # distance from the target calculation
            goal_dist = math.sqrt((x_pos-x_goal)**2 + (y_pos - y_goal)**2)
            
            # psi target calculation
            psi_target = (math.atan2((y_goal-y_pos), (x_goal-x_pos)))*180/np.pi # deg
            if psi_target < 0:
                psi_target = psi_target + 360
            
            # Questo blocco serve nel caso lo psi target sia vicino allo 0
            psi_target_new = psi_target
            if 0 < psi_target and psi_target <= 180 and 180 < psi and psi <= 360:
                psi_target_new = psi_target + 360
            if 180 < psi_target and psi_target <= 360 and 0 < psi and psi <= 180:
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

            kal.fu = force_z #Change the force input in kalman               
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
                kal.fl = l_pwm / (100/0.1)
                '''
                if l_pwm >= 0:
                    p2.ChangeDutyCycle(l_pwm)
                else:
                    p2i.ChangeDutyCycle(-l_pwm)
                '''

                r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                kal.fr = r_pwm / (100/0.1)
                
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
                kal.fr = 0
                kal.fl = 0
                '''
                p2.ChangeDutyCycle(l_pwm)
                p2i.ChangeDutyCycle(-l_pwm)
                p3.ChangeDutyCycle(r_pwm)
                p3i.ChangeDutyCycle(-r_pwm)
                '''
                
            # Cambia il valore delle PWM globali e mandale al blimp
            global_.pwm_left = l_pwm
            global_.pwm_right = r_pwm
            global_.pwm_up = z_pwm
 
            stringa_g = [tempo, l_pwm, r_pwm, z_pwm, goal_dist, psi_target, z_target]
            res_g = " ".join([str(j) for j in stringa_g])
            g.write(res_g)
            g.write("\n")

                
            #print("dist, psi e psi_target = ", dist_dist, psi*180/np.pi, psi_target*180/np.pi)
            print("psi, psi_target e goal distance = ", psi, psi_target, goal_dist)
            print("l_pwm, r_pwm e z_pwm = ", l_pwm, r_pwm, z_pwm)
            print("Accelerazioni ruotate", acc_abs)
            '''
            # Invio della posizione per la stampa
            zipcode = 10001
            mes_x = x_pos
            mes_y = y_pos
            #socket.send_string(f"{zipcode} {mes_x} {mes_y}")
            '''

            time_zero_1Hz = time.perf_counter() 
                

if __name__ == "__main__":
    f = open("log_Blimp_00_data.txt", "w") # Here Timestep, IMU, UWB and SONAR data are stored
    g = open("log_Blimp_navigation.txt", "w") # timestep, l_pwm, r_pwm, z_pww, distance, psi and psi_target
    l = open("kalman_state_vector.txt", "w") # The points calculated by the A* 
   
   
     # 1. Madgwick

     # 2. Kalman


    '''
    Ricevi gli aggiornamenti e fai i calcoli il più velocemente possibile. 
    Poi aggiorna le forze una volta al secondo nel modello del kalman
    e anche veramente nel modello del blimp. 
    Salva uno storico di log delle info dei sensori e dello stato. 
    '''
    try:    
        main()  

    except TypeError: 
        print("Crashed :/")
       
        print ('Serial port closed')
        print("Motors stopped") 
        f.close()
        l.close()
        g.close()
        logging.exception("message")

    except ValueError: 
        print("Crashed :/") 
        f.close()
        l.close()
        g.close()
        logging.exception("message")



    except KeyboardInterrupt:
        print("Interrupted")
        f.close()
        l.close()
        g.close()
        logging.exception("message")
        
        '''
        #  Wait for next request from client
        message = socket.recv_string()
        zip_code, tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, mesg = message.split(",")
        print("Received Infos for the execution of the navigation")
        print(zip_code, tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, mesg)
        '''    
        #  Do some 'work'
       
        
