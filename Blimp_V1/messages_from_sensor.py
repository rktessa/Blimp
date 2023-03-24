import serial
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map
#, kalman_blimp 
import time
from quaternion import Quaternion
import numpy as np 


##################################################################
# Codice per provare a usare UWB
mesg = {}

#SET OUTPUT MESSAGE
hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
rl = ReadLine(hser)

q0 = Quaternion(0, 0, 0, 1)
mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)


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
            #print(roll, pitch, yaw)

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



