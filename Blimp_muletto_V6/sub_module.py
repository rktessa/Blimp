import os
import sys
import numpy as np
import zmq
import time

# Si definisce anche qui il context, PUBLISHER per 
# spedire le informazioni
context = zmq.Context()
print("Mando?")
socket = context.socket(zmq.PUB)
socket.bind("tcp://192.168.1.175:5557")
# Qui posso creare pub che spara al sub del raspberry per controllare i motori
# e funziona da dentro la funzione 

def pwm_func():
    from global_ import pwm_left, pwm_right, pwm_up
    
    # Parte della funzione che si occupa di mandare il messaggio, quando viene invocato nel main
    zip_filter = "10001"
    socket.send_string(f"%s %f %f %f"% (zip_filter, pwm_left, pwm_right, pwm_up))
    print("Mandato")
    #context.term()
    


