#
#   Hello World client in Python
#   Connects REQ socket to tcp://localhost:5555
#   Sends "Hello" to server, expects "World" back
#
import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import calibration
from blimp_class import imu_to_uwb, rotation_UWB
import numpy as np
import math
from numpy.linalg import norm, inv
import serial
import time
#import select
#import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate
#import cv2
import logging
import zmq
import time

# Classe per UWB
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
    
    return distance, flag



# Parte per la connesione al Server
context = zmq.Context()

#  Socket to talk to server
print("Connecting to hello world server...")
socket = context.socket(zmq.PUB)
socket.bind("tcp://192.168.1.104:5556")

HCtrig=8     
HCecho=7     

IO.setwarnings(False)           
IO.setmode(IO.BCM)        # numerazione bcm dei pin
IO.setup(HCtrig,IO.OUT)   # setup ultrasuoni
IO.setup(HCecho,IO.IN)   

i2c = board.I2C()  # uses board.SCL and board.SDA
icm = ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ


# Mando come REQ le informazioni al server, che fa tutti i calcoli
# E rimanda direttamente anche la pwm per i motori
# Poi sono io che aggiorno ogni 2 Hz la misura proveniente dal sonar, mentre
# la mando in continuazione e anche sono io che ogni secondo cambio la pwm
# fisica dei motori. 
if __name__ == "__main__":
    # IMU 

    # Codice per provare a usare UWB
    mesg = {}
    mesg0 = {}
    # SET OUTPUT MESSAGE
    zip_code = "Blimp"
    hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
    rl = ReadLine(hser)
    
    # request = 0
    zero = time.perf_counter()
    time_zero_Sonar = time.perf_counter()

    mesg0 = rl.readline().decode("utf-8")
    ts = mesg0.split(" ")
    if (len(ts)!=25):
        mesg0 = rl.readline().decode("utf-8")
        ts = mesg0.split(" ")

    distance, flag = Sonar()

    if flag:
        z_pos = distance
        
    try: 
        while True:

            mesg = rl.readline().decode("utf-8")
            
            ts = mesg.split(" ")
            if (len(ts)!=25):
                mesg = rl.readline().decode("utf-8")
                ts = mesg.split(" ")

            tempo = time.perf_counter()
            raw_acc, raw_gyr, raw_mag = icm.acceleration,icm.gyro, icm.magnetic
            #Aquisizione magnetometro e calibrazione dei dati:
            # magnet = calibration(raw_mag)

            # acc, gyro, magnet = imu_to_uwb(raw_acc,raw_gyr,mag)

            if time.perf_counter() - time_zero_Sonar >= 0.5:
                distance, flag = Sonar()

                if flag:
                    z_pos = distance
                
                time_zero_Sonar = time.perf_counter()
            
            # Invio dei dati al server che fa calcoli e simualazione
            socket.send_string(f"%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s" % (zip_code, str(tempo),str((raw_acc[0])), str((raw_acc[1])),str((raw_acc[2])),
                                                                          str((raw_gyr[0])), str((raw_gyr[1])), str((raw_gyr[2])), str((raw_mag[0])),
                                                                            str((raw_mag[1])), str((raw_mag[2])), mesg, str((z_pos))))
        
            #  Get the reply.
            # message = socket.recv()
            freq = 1.0/ (time.perf_counter() - zero)
            zero = time.perf_counter()
            # request = request + 1
            # print(f"Received reply {request} [ {message} ]")
            print("frequenza = ", freq)
            #time.sleep(1)


    except KeyboardInterrupt:
            print("Fine acquizione")
            hser.close()
            print ('Serial port closed')

    except TypeError: 
        print("Crashed :/")
        hser.close()
        print ('Serial port closed')
        logging.exception("message")

    except ValueError: 
        print("Crashed :/")
        hser.close()
        print ('Serial port closed')
        logging.exception("message")