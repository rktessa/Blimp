import serial
import time
from blimp_class import TDoA, TDoA_dt, reject_outliers #, kalman_blimp
import numpy as np 

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

tempi_dt = np.zeros((1,6))



try:
    mesg0 = rl.readline().decode("utf-8")
    #print(mesg)
    ts = mesg0.split(" ")
    dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
    #print(dt_uwb)
    tempi_dt[0,:] = dt_uwb
    
    while True:
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

except KeyboardInterrupt:
    hser.close()
    print ('Serial port closed')

