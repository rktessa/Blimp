# UWB CHE FUNZIONA E SALVA I DATI GIUSTI

import serial
import numpy as np
import time
from blimp_class import TDoA, TDoA_dt, reject_outliers, Madgwick,calibration, psi_map #, kalman_blimp
import board
from adafruit_icm20x import ICM20948, MagDataRate
from quaternion import Quaternion

i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ

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

mesg = {}

#mag_file = open("data_UWB_V30.txt","w")
#data_string = ""

#SET OUTPUT MESSAGE
hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
rl = ReadLine(hser)

q0 = Quaternion(0, 0, 0, 1)
mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=0.35)
time_zero_mad = time.perf_counter()

tempi_dt = np.zeros((1,6))
#x_cord = np.empty((1,0))
#y_cord = np.empty((1,0))
#z_cord = np.empty((1,0))

mesg0 = rl.readline().decode("utf-8")
#log_file = open("log_V30.txt","a")
#log_file.write(mesg0)

ts = mesg0.split(" ")

if len(ts) != 25:
  mesg0 = rl.readline().decode("utf-8")
  ts = mesg0.split(" ")
  
dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
    #print(dt_uwb)
tempi_dt[0,:] = dt_uwb


time_start = time.perf_counter()
try:
  while 1:
      time_current = time.perf_counter()
      mesg = rl.readline().decode("utf-8")
      
      #log_file = open("log_V30.txt","a")
      #log_file.write(mesg)
      #print(mesg)
          
      ts = mesg.split(" ")
      
      if len(ts) != 25:
        mesg = rl.readline().decode("utf-8")
        ts = mesg.split(" ")
  
      
      #x_cord = np.append(x_cord, x_pos_uwb)
      #y_cord = np.append(y_cord, y_pos_uwb)
      #z_cord = np.append(z_cord, z_pos_uwb)
  
      
      
      #print(dt_uwb)
      #dt_uwb = np.mean(reject_outliers(tempi_dt))
      
      if (time_current-time_start) >= 0.5:
        x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
        dt_new = np.reshape(dt_new, (1,6))
        tempi_dt = np.append(tempi_dt,dt_new, axis=0)
        for i in range(6) :
            dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=2))
        
        print("coordinate = ", [x_pos_uwb, y_pos_uwb, z_pos_uwb])

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
        
        time_start = time.perf_counter()
      #data_string = str(x_pos_uwb) + ", " + str(y_pos_uwb) + ", " + str(z_pos_uwb) + "\n"
      #mag_file.write(data_string)
      
  #print(tempi_dt)

except KeyboardInterrupt:
    hser.close()
    print ('Serial port closed')
   