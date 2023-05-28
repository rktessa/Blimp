# Simple code for read all the data from the IMU
# 10/05/23
# Riccardo Tessarin

import numpy as np
import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import calibration
from quaternion import Quaternion
import math
import time
import board
from adafruit_icm20x import ICM20948, MagDataRate

i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ


if __name__ == "__main__":
    # Create a file to write the output
    f = open("log_IMU_static.txt", "w")
    print("Inizio registrazione")
    while  1:
        try:
            tempo = time.perf_counter()
            raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
            #Aquisizione magnetometro e calibrazione dei dati:
            mag = calibration(raw_mag)
            stringa = [tempo, raw_acc[0], raw_acc[1], raw_acc[2], raw_gyr[0], raw_gyr[1], raw_gyr[2], mag[0], mag[1], mag[2]]
            res = " ".join([str(i) for i in stringa])
            f.write(res)
            f.write("\n")
        except KeyboardInterrupt:
            print("Fine acquizione")
            f.close()
