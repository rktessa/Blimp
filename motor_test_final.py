import warnings
import socket
import RPi.GPIO as IO 
#from blimp_class import Madgwick, calibration, PID_Controller, trilateration, TDoA #, kalman_blimp
import numpy as np
import math
#from numpy.linalg import norm
#from quaternion import Quaternion
#import Blimp.serial as serial
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate


#Definizione di tutti i pin per uso
#Definizione di tutti i pin per uso
m1 = 17 #sinistro
m1_i = 27
m2 = 13  #destro
m2_i = 19
m3 =  20 #sotto
m3_i =  21

IO.setwarnings(False)           
IO.setmode(IO.BCM)        #numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori  
IO.setup(m1_i,IO.OUT)
IO.setup(m2,IO.OUT)
IO.setup(m2_i,IO.OUT)
IO.setup(m3,IO.OUT)
IO.setup(m3_i,IO.OUT)  


p1 = IO.PWM(m1,100)       #inizializzazione pwm        
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

def main():
    while 1:
        pwm = input("Input a PWM")
        print("La pwm impostata è: ", pwm)
        p3.ChangeDutyCycle(float(pwm))


if __name__ == "__main__":
    
    try:
        main()    
    
    except KeyboardInterrupt:
        print("Interrupted")
        p3.ChangeDutyCycle(0)
        
        
        
        

