import sys
import zmq
import RPi.GPIO as IO 
import logging

#  Socket to talk to servers
# Questo è il codice che riceve le informazioni 
# nel blimp e cambia la pwm
context = zmq.Context()
socket = context.socket(zmq.SUB)

print("Collecting updates from PC  server...")
socket.connect("tcp://192.168.1.175:5557")
print("Mi sono connesso al PC per ricevere le PWM")

# Subscribe to zipcode, default is NYC, 10001
zip_filter = "10001"
socket.setsockopt_string(zmq.SUBSCRIBE, zip_filter)

# Definizione di tutti i pin per uso per controllare i motori
m1 = 17 # sinistro
m1_i = 27
m2 = 13  # destro
m2_i = 19
m3 =  20 # sotto
m3_i =  21

IO.setwarnings(False)           
IO.setmode(IO.BCM)        # numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori  
IO.setup(m1_i,IO.OUT)
IO.setup(m2,IO.OUT)
IO.setup(m2_i,IO.OUT)
IO.setup(m3,IO.OUT)
IO.setup(m3_i,IO.OUT)  

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

def main():
    while True:

        string = socket.recv_string()
        zipfilter, pwm1, pwm2, pwm3 = string.split()
        pwm1 = float(pwm1)
        pwm2 = float(pwm2)
        pwm3 = float(pwm3)
        print(pwm1, pwm2, pwm3)
        if pwm1 >= 0:
            #p1.ChangeDutyCycle(pwm1)
            pippo = 1
        else:
            pwm1 = -1*pwm1
            #p1i.ChangeDutyCycle(pwm1)

        if pwm2 >= 0:
            #p2.ChangeDutyCycle(pwm2)
            pippo = 1
        else:
            pwm2 = -1*pwm2
            #p2i.ChangeDutyCycle(pwm2)

        if pwm3 >= 0:
            #p3.ChangeDutyCycle(pwm3)
            pippo = 1
        else:
            pwm3 = -1*pwm3
            #p3i.ChangeDutyCycle(pwm3)

        #print((f"The received pwm is " f"{zipfilter} {pwm1} {pwm2} {pwm3}"))
        print(pwm1, pwm2, pwm3)
if __name__ == "__main__":
    
    try:
        main()    
    
    except KeyboardInterrupt:
        print("Interrupted, I stop the motors")
        p1.ChangeDutyCycle(0)
        p2.ChangeDutyCycle(0)
        p3.ChangeDutyCycle(0)
        