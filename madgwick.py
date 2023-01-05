# -*- coding: utf-8 -*-
"""Codice che implementa per il blimp il madgwick,
un algoritmo usato per stimare l'heading del blimp 
mentre è in movimento. """


"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
"""IMPOSTAZIONI CODICE DI FELETTO:
 dt = 1e-2
 q0 = sp.spatial.transform.Rotation.from_quat([1, 0, 0, 0])
 q = (q0 * rot.inv()).as_quat()[[3, 0, 1, 2]]
    madgwick = pejo.Madgwick(dt, beta=1, q=q)  

"""



import warnings
from blimp_class import Madgwick, calibration, orientation_initial
import numpy as np
from numpy.linalg import norm
from quaternion import Quaternion
import time
import board
from adafruit_icm20x import ICM20948, MagDataRate
#import scipy as sp
#import scipy.linalg
#import scipy.spatial


i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ



if __name__ == "__main__":
    print("Start of Madgwick: setting 0 angle pose")
    print("Wait 10 seconds")
    roll_0, pitch_0, yaw_0 = orientation_initial(icm.acceleration,icm.gyro, icm.magnetic)
    
    print("Start of live output")
    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1) # Inizializzazione della classe Madgwick
    
    time_zero = time.perf_counter()
    
    while 1:
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
        #print("res = ", str(quaternion))
        roll, pitch, yaw = quat.to_euler123()
        roll = roll - roll_0
        pitch = pitch-pitch_0
        yaw = yaw - yaw_0
        print(roll, pitch, yaw)
        
