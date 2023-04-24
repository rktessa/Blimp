# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y, z directions,
# the acceleration in the same directions and the yaw orientation and angular velocity.
# Jan 23
# Riccardo Tessarin

import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map, Astar
import numpy as np
import math
from numpy.linalg import norm, inv
from quaternion import Quaternion
import serial
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate
import cv2
import logging
from matplotlib import pyplot as plt


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
## FINE 

def blimp_to_world_rf():

    ##################################################################
    # Codice per provare a usare UWB
    # mesg_0 = {}

    # SET OUTPUT MESSAGE
    #hser0 = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    #rl0 = ReadLine(hser0)

    ######################################################


    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []
    x = np.empty((1,0))
    y = np.empty((1,0))
    z = np.empty((1,0))

    # per UWB
    tempi_dt = np.zeros((1,6))

    try:
        #print("QUI")
        mesg0 = rl.readline().decode("utf-8")
        ts = mesg0.split(" ")
        if (len(ts)!=25):
          mesg0 = rl.readline().decode("utf-8")
          ts = mesg0.split(" ")
        dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
        tempi_dt[0,:] = dt_uwb

        time_zero = time.perf_counter()
        time_start = time.perf_counter()
        time_start2 = time.perf_counter()
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
            
            psi = yaw*180/np.pi
            if psi < 0:
                psi = psi + 360
            yaw_mapped = psi_map(psi)
            roll_vec.append(roll)
            pitch_vec.append(pitch)
            yaw_vec.append(yaw_mapped*np.pi/180)
            
            # Misuro con UWB la posizione nel piano  frattempo 
            mesg0 = rl.readline().decode("utf-8")
            
            ts = mesg0.split(" ")
            if (len(ts)!=25):
              mesg0 = rl.readline().decode("utf-8")
              ts = mesg0.split(" ")
            
            if (time.perf_counter()-time_start2) >= 0.5:
                x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)

                if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                    x = np.append(x, x_pos_uwb)
                    y = np.append(y, y_pos_uwb)
                    z = np.append(z, z_pos_uwb)
                    print(x_pos_uwb,y_pos_uwb,z_pos_uwb)
            
                dt_new = np.reshape(dt_new, (1,6))
                tempi_dt = np.append(tempi_dt,dt_new, axis=0)
                
                for i in range(6) :
                    dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
                time_start2 = time.perf_counter()

            # Save the UWB results in a .txt file
            # data_string = str(x_pos_uwb) + " ," + str(y_pos_uwb) + " ," + str(z_pos_uwb) + " ," + str(yaw) + "\n"
            # mag_file.write(data_string)
            
    
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')

    quat_final = quat # the last quaternion is stored as input for the madgwick later
    roll_0 = sum(roll_vec[-20 :])/20 # perform the mean of the last 40 element
    pitch_0 = sum(pitch_vec[-20 :])/20
    yaw_0 = sum(yaw_vec[-20 :])/20

    x_0 = sum(x[-20 :])/20 # perform the mean of the last 40 element
    y_0 = sum(y[-20 :])/20
    z_0 = sum(z[-20 :])/20
    
    # Ora usando i dati creati vado a fare una LLS estimation
    # y = a + b*x
    # a = (np.sum(y) * np.sum(x**2)- np.sum(x)*np.sum(x*y))/ (len(x)*np.sum(x**2) - np.sum(x)**2)
    # b = ( len(x)* np.sum(x*y) - np.sum(x)*np.sum(y)) /(len(x)*np.sum(x**2)- np.sum(x)**2) 

    # The error of my model
    
    # ang_rad = np.arctan(b) # the inclination of the line calculated in rad with uwb
    # delta = yaw_0 - ang_rad

    # Prima di finire chiudi la serial port
    

    return yaw_0, quat_final, x_0, y_0, z_0 # These are the values of initial angles
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
    
    #print(counter)
    #z_sonar = round(distance, 3)
    
    return distance, flag











def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z must be convertible to shape ({}, 1)'.format(dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z








# Definition of the matrices used in 
# Kalman filter
class kalman_blimp:
    c = 1.60/2.0 # m half lenght of blimp on x axis
    b = 0.40/2.0 # m half lenght of blimp on y axis
    dt = 1 
    xR = 0.0675 # m distance of R motor from CG
    xL = 0.0675 # m distance of R motor from CG
    m = 0.2713 # kg total mass of airship
    I_z = m *(c*c + b*b)/5 # inertia
    Fl = 0.0 # N forces of the motors
    Fr = 0.0
    Fu = 0.0
    x_pos = 0.0
    y_pos = 0.0
    z_pos  = 0.0 
    phi_pos = 0.0


    def __init__(self, dt=None, Fl= None, Fr=None, Fu=None, x_pos=None, y_pos=None, z_pos=None, phi_pos=None):
        """
        Initialize the class with the given parameters.
        :param dt: The sample period
        :return:
        """
        if dt is not None:
            self.dt = dt
        if Fl is not None:
            self.Fl = Fl
        if Fr is not None:
            self.Fr = Fr
        if Fu is not None:
            self.Fu = Fu
        if x_pos is not None:
            self.x_pos = x_pos
        if y_pos is not None:
            self.y_pos = y_pos
        if x_pos is not None:
            self.z_pos = z_pos
        if x_pos is not None:
            self.phi_pos = phi_pos
       
    def initialization(self):
        
        # State vector is x = [x, x'', y, y'', z, z'', phi, phi' ]
        # at the beginning is initialized with the value for position
        # measured by sonar, madgwick and uwbz
        x = np.transpose(np.array([self.x_pos, 0.0, self.y_pos, 0.0, self.z_pos, 0.0, self.phi_pos, 0.0]))

        F = np.array([[1.0, (self.dt*self.dt)/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    [0.0, 0.0, 1.0, (self.dt*self.dt)/2.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 0.0, 1.0, (self.dt*self.dt)/2.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, self.dt],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        '''B = np.array([[0.0, 0.0, 0.0],
                    [1.0/self.m, 1.0/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])'''
        B = np.array([[0.0, 0.0, 0.0],
                    [math.cos(self.phi_pos)/self.m, math.cos(self.phi_pos)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [math.sin(self.phi_pos)/self.m, math.sin(self.phi_pos)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])

        

        u_t =  np.transpose(np.array([self.Fl, self.Fr, self.Fu])) #input vector depending on force

        
        
        #State covariance matrix
        # maybe in P it is better to put a rad and rad/s value for phi and phi vel.
        P = np.diag([0.0064, 0.0005, 0.0030, 0.0007, 0.00034, 0.002, 0.00015, 0.0000012])
        # Q calcolata a caso al momento
        Q =  np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ])*0.001

        # H is an Identity matrix beacuse all the quantities are directly measured by
        # a dedicated sensor
        H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])

        R = np.diag([0.0064, 0.0005, 0.0030, 0.0007, 0.00034, 0.002, 0.00015, 0.0000012])
        
        return F, B, u_t ,x, P, Q, H, R

    def predict_kal(self, x, P, F=1, Q=0, u=0, B=1, alpha=1.):
        """
        Predict next state (prior) using the Kalman filter state propagation
        equations.

        Parameters
        ----------

        x : numpy.array
            State estimate vector

        P : numpy.array
            Covariance matrix

        F : numpy.array()
            State Transition matrix

        Q : numpy.array, Optional
            Process noise matrix


        u : numpy.array, Optional, default 0.
            Control vector. If non-zero, it is multiplied by B
            to create the control input into the system.

        B : numpy.array, optional, default 0.
            Control transition matrix.

        alpha : float, Optional, default=1.0
            Fading memory setting. 1.0 gives the normal Kalman filter, and
            values slightly larger than 1.0 (such as 1.02) give a fading
            memory effect - previous measurements have less influence on the
            filter's estimates. This formulation of the Fading memory filter
            (there are many) is due to Dan Simon

        Returns
        -------

        x : numpy.array
            Prior state estimate vector

        P : numpy.array
            Prior covariance matrix
        """

        if np.isscalar(F):
            F = np.array(F)

        phi = x[6]
        

        B2 = np.array([[0.0, 0.0, 0.0],
                    [math.cos(phi)/self.m, math.cos(phi)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [math.sin(phi)/self.m, math.sin(phi)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])
        
        #x_pre = np.dot(R,F)
        #x = np.dot(x_pre, x) + np.dot(B, u)
        
        x = np.matmul(F, x) + np.matmul(B2, u)
        P = (alpha * alpha) * np.matmul(np.matmul(F, P), F.T) + Q
        # write in a csv file the x at each iteration
        return x, P
    
    def update_kal(self, x, P, z, R, H=None, return_all=False):
        """Add a new measurement (z) to the Kalman filter. If z is None, nothing
        is changed.

        This can handle either the multidimensional or unidimensional case. If
        all parameters are floats instead of arrays the filter will still work,
        and return floats for x, P as the result.

        update(1, 2, 1, 1, 1)  # univariate
        update(x, P, 1



        Parameters
        ----------

        x : numpy.array(dim_x, 1), or float
            State estimate vector

        P : numpy.array(dim_x, dim_x), or float
            Covariance matrix

        z : (dim_z, 1): array_like
            measurement for this update. z can be a scalar if dim_z is 1,
            otherwise it must be convertible to a column vector.

        R : numpy.array(dim_z, dim_z), or float
            Measurement noise matrix

        H : numpy.array(dim_x, dim_x), or float, optional
            Measurement function. If not provided, a value of 1 is assumed.

        return_all : bool, default False
            If true, y, K, S, and log_likelihood are returned, otherwise
            only x and P are returned.

        Returns
        -------

        x : numpy.array
            Posterior state estimate vector

        P : numpy.array
            Posterior covariance matrix

        y : numpy.array or scalar
            Residua. Difference between measurement and state in measurement space

        K : numpy.array
            Kalman gain

        S : numpy.array
            System uncertainty in measurement space

        log_likelihood : float
            log likelihood of the measurement"""

        #pylint: disable=bare-except

        if z is None:
            if return_all:
                return x, P, None, None, None, None
            return x, P

        if H is None:
            H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])

        if np.isscalar(H):
            H = np.array([H])

        Hx = np.atleast_1d(np.matmul(H, x))
        z = reshape_z(z, Hx.shape[0], x.ndim)

        # error (residual) between measurement and prediction
        y = z - Hx

        # project system uncertainty into measurement space
        S = np.matmul(np.matmul(H, P), H.T) + R
        #print(S)
        #print(1./S)
        #print(inv(S))


        # map system uncertainty into kalman gain
        try:
            K = np.matmul(np.matmul(P, H.T), inv(S))
        except:
            # can't invert a 1D array, annoyingly
            K = np.matmul(np.matmul(P, H.T), 1./S)


        # predict new x with residual scaled by the kalman gain
        x = x + np.matmul(K, y)

        # P = (I-KH)P(I-KH)' + KRK'
        KH = np.matmul(K, H)

        try:
            I_KH = np.eye(KH.shape[0]) - KH
        except:
            I_KH = np.array([1 - KH])
        #P = np.matmul(np.matmul(I_KH, P), I_KH.T) + np.matmul(np.matmul(K, R), K.T)
        P = np.matmul(I_KH, P)

        if return_all:
            # compute log likelihood
            log_likelihood = np.logpdf(z, np.matmul(H, x), S)
            return x, P, y, K, S, log_likelihood
        return x, P

     
def main():

    #ORIENTAZIONE NEL GLOBAL REFERENCE FRAME
    # Qui posso lanciare tutte le operazioni preliminari
    # per il corretto funzionamento del codice

    # Primo step, calcolo della orientazione iniziale del 
    # dirigibile nello spazio
    yaw_0, quat_final, x_pos_init, y_pos_init, z_pos_init = blimp_to_world_rf() #==> delta, ang rad
    # Plotting the final results
    print("yaw_0  del blimp per il Madgwick= ", yaw_0*180/np.pi)

    #############################################################################################################
    # Path Planning prova
    
    print("Inizio path planning")
    img = cv2.flip(cv2.imread("/home/piblimp/Blimp_23/Blimp_V2/lab_meccatronica_V2.png"),0)
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20, 20)))
    img = img.astype(float)/255.



    start = (int(x_pos_init*100),int(y_pos_init*100)) ## MY case
    goal = (100, 500)

    
    astar = Astar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=10) #10 � 1 metro nella realt�
    traj = np.array(path)/100.0
    print(traj) # ==> questo � il percorso da seguire
    #############################################################################################################

    # indice punto della traiettoria
    val = 0
    
    # inizializzo x_pos e y_pos dell'UWB
    x_pos = x_pos_init
    y_pos = y_pos_init


    





    ##############################################
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

        # Qui posso lanciare tutte le operazioni preliminari
        # per il corretto funzionamento del codice

        # Calcolare la traiettoria da seguire con il path planner

        # Madgwick initialization
        q0 = quat_final
        mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35) 
        
        # PID initialization
        dist_pid = PID_Controller(kp= 1, ki = 0, kd = 0, max_signal =12, sample_rate=1.0, target=0.0)
        psi_pid = PID_Controller(kp= 1, ki = 0, kd = 0, max_signal =12, sample_rate=1.0, target=0.0)
        z_pid = PID_Controller(kp= 1, ki = 0, kd = 0, max_signal =12, sample_rate=1.0, target=0.0)

        dist_target = 0.01 # m distance the blimp want to minimize
        
        z_target = 0.5  # Mantenimento di una data quota

        # Kalman initialization
        # kal = kalman_blimp()

        # Contatore per le misure fatte a 1 HZ per IMU e Sonar
        zero_measure_time = time.perf_counter()
        zero_measure_time2 = time.perf_counter()
        # SET ZERO TIME, FUNDAMENTAL BEFORE THE WHILE LOOP
        time_zero = time.perf_counter()

        # Initialize the sonar for a first value
        z_sonar, flag = Sonar()
        
        if flag:
          z_pos = z_sonar

        #########################
        # KALMAN self, dt=None, Fl= None, Fr=None, Fu=None, x_pos=None, y_pos=None, z_pos=None, phi_pos=None
        kal = kalman_blimp(dt =0.01 , Fl = 0,  Fr = 0 , Fu = 0, phi_pos=ang_rad, x_pos=x_pos, y_pos=y_pos, z_pos=z_pos)
        # When the class is initialized, here I can insert the intial coordinate
        # for position and orientation in Global reference frame. 
        
        
        
        F, B, u_t ,x, P , Q, H, R = kal.initialization() 
        
    
    
    # Loop to simulate the evolution of the dynamics and
    # store the values for visualization
        while 1: 
            
            ##### KALMAN PREDICT E UPDATE ##############################
            x, P = kal.predict_kal(x=x, P=P, F=F, Q=Q, u=u_t, B=B)
            
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(icm.magnetic)
            # Creazione vettore input per classe madgwick
            accelerometer, gyroscope, magnetometer = np.asarray(icm.acceleration), np.asarray(icm.gyro), np.asarray(mag)
            time_fine = time.perf_counter()
            # setting the variable frequency of updateof Madgwick alghorithm
            mad.samplePeriod = time_fine - time_zero
            kal.dt = time_fine - time_zero
            quaternion = mad.update(gyroscope, accelerometer, magnetometer)
           
            time_zero = time.perf_counter() #ORIGINAL TIME ZERO FOR MADGWICK ALONE
            quat = Quaternion(quaternion)
            roll, pitch, yaw = quat.to_euler123()
            psi_mapped = psi_map(yaw*180/np.pi) # deg
            
            psi = psi_mapped * np.pi/180 - delta # rad
            if psi < 0:
                psi = psi + 2*np.pi
            

            #######################
            # UWB ALWAYS WORKING
            ########################
            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
            
            if (len(ts)!=25):
              mesg = rl.readline().decode("utf-8")
              ts = mesg.split(" ")
                
            x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)


            dt_new = np.reshape(dt_new, (1,6))
            tempi_dt = np.append(tempi_dt,dt_new, axis=0)
            for i in range(6) :
                dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
            
            if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0  and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                x_pos = x_pos_uwb
                y_pos = y_pos_uwb
                # print("coordinate = ", [x_pos_uwb, y_pos_uwb, z_pos_uwb])
            
                   
            #print("predict x = ", x)
            z = np.array([[x_pos, accelerometer[0], y_pos, accelerometer[1], z_pos, accelerometer[2], psi, gyroscope[2]]]).T
            
            x, P = kal.update_kal(x, P, z, R, H)

            #print(x)
            
            # Uso i valori calcolati con il Kalman
            x_pos = x[0]
            y_pos = x[2]
            #z_pos = x[4] per ora usiamo solo quella del Sonar
            psi =x[6]

            #print(x_pos,y_pos,psi)
            ###############################
            ###############################
            # dist and phi pid block of code
            dist_dist = math.sqrt((x_pos-traj[val][0])**2 + (y_pos - traj[val][1])**2)
            
            # z_target = 1.20  # Mantenimento di una data quota
            
            # Other sensors, they cycle at 1 HZ for refresh the PID and the motor values
            #Ciclo if solo per il sonar
            current_measure_time2 = time.perf_counter()

            if (current_measure_time2 - zero_measure_time2) >= 1.0: 
                z_sonar, flag = Sonar()
                zero_measure_time2 = time.perf_counter()
                if flag:
                  z_pos = z_sonar

            current_measure_time = time.perf_counter()
            if (current_measure_time - zero_measure_time) >= 1.0:
                
                #z_pos = 1
                # x_pos, y_pos, psi and z_pos are our measurements
                 # Manipolazione della psi per gestire la navigazione con un range di misure che va da 0 a 2 pi
                psi_target = math.atan2((traj[val][1]-y_pos), (traj[val][0]-x_pos)) # rad
                if psi_target < 0:
                    psi_target = psi_target + 2*np.pi
                # 
                psi_target_new = psi_target
                if 0 < psi_target and psi_target <= np.pi/2 and 3/2*np.pi < psi and psi <= 2*np.pi:
                    psi_target_new = psi_target + 2*np.pi
                if 3/2*np.pi < psi_target and psi_target <= 2*np.pi and 0 < psi and psi <= np.pi/2:
                    psi_target_new = psi_target - 2*np.pi
                psi_target = psi_target_new
                
                ####################################################
                # SET INITIAL TARGET FOR PID OBJECT CLASS
                z_pid.set_new_target(z_target)
                psi_pid.set_new_target(psi_target)
                dist_pid.set_new_target(dist_target)
                #####################################################

                

                ##############################################
                # Z_PID block of code 
                ##############################################

                # z_pid.sample_rate = time_fine - time_zero # al momento � costante
                signal_z = z_pid.adjust_signal(z_pos)
                force_z = z_pid.get_force_z(signal_z) # Questa serve per il Kalman
                kal.Fu = force_z
                z_pwm = z_pid.pwm_z_motor(force_z) # Questa serve per i motori
                
                print("z_pos = ", z_pos)
                print("z_target = ", z_target)
                print("z_pwm = ", z_pwm)            
                '''if z_pwm >= 0:
                    p1.ChangeDutyCycle(z_pwm)
                else:
                    p1i.ChangeDutyCycle(-z_pwm)'''
            
                # Per variare il sample rate del PID controller durante la run
                # phi_pid.sample_rate = time_fine - time_zero # al momento � costante
                # dist_pid.sample_rate = time_fine - time_zero
                # time_zero = time.perf_counter()

                signal_psi = psi_pid.adjust_signal(psi)
                
                signal_dist = dist_pid.adjust_signal(dist_dist)
                
                force_l, force_r = dist_pid.get_force_lateral(signal_dist, signal_psi) # Questa serve per il Kalman
                kal.Fl = force_l
                kal.Fr = force_r
                l_pwm = dist_pid.pwm_L_motor(force_l) # Questa serve per i motori
                
                '''if l_pwm >= 0:
                    p2.ChangeDutyCycle(l_pwm)
                else:
                    p2i.ChangeDutyCycle(-l_pwm)'''

                r_pwm = dist_pid.pwm_R_motor(force_r) # Questa serve per i motori
                
                print("psi = ", psi*180/np.pi)
                print("psi_target = ", psi_target*180/np.pi)
                print("dist = ", dist_dist)
                print("l_pwm = ", l_pwm)
                print("r_pwm = ", r_pwm)
                
                '''if r_pwm >= 0:
                    p3.ChangeDutyCycle(r_pwm)
                else:
                    p3i.ChangeDutyCycle(-r_pwm)'''
                zero_measure_time = time.perf_counter()

            if dist_dist < 0.1:
                    val = val + 1
                    # aumenti un array trajectory
                    z_pid.set_new_target(z_target)
                    psi_target = math.atan2((traj[val][1]-y_pos), (traj[val][0]-x_pos)) # rad
                    if psi_target < 0:
                        psi_target = psi_target + 2*np.pi
                    psi_pid.set_new_target(psi_target)
                    dist_pid.set_new_target(dist_target)
                    print("val = ", val)
                
    except KeyboardInterrupt:
        hser.close()
        print ('Serial port closed')





if __name__ == "__main__":

    # Codice per provare a usare UWB
    mesg = {}

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




