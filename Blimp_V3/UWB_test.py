# UWB CHE FUNZIONA E SALVA I DATI GIUSTI

import serial
import numpy as np
import time
from blimp_class import TDoA, TDoA_dt, reject_outliers #, kalman_blimp

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


# time_start = time.perf_counter()
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
  
      x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)
      #x_cord = np.append(x_cord, x_pos_uwb)
      #y_cord = np.append(y_cord, y_pos_uwb)
      #z_cord = np.append(z_cord, z_pos_uwb)
  
  
      dt_new = np.reshape(dt_new, (1,6))
      tempi_dt = np.append(tempi_dt,dt_new, axis=0)
      for i in range(6) :
          dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=5))
      #print(dt_uwb)
      #dt_uwb = np.mean(reject_outliers(tempi_dt))
      #dt_uwb = dt_new
      #if (time_current-time_start) >= 1:
      print("coordinate = ", [x_pos_uwb, y_pos_uwb, z_pos_uwb])
        #time_start = time.perf_counter()
      #data_string = str(x_pos_uwb) + ", " + str(y_pos_uwb) + ", " + str(z_pos_uwb) + "\n"
      #mag_file.write(data_string)
      
  #print(tempi_dt)

except KeyboardInterrupt:
    hser.close()
    print ('Serial port closed')
    log_file.close()
    mag_file.close()