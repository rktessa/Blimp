import os
import sys
import numpy as np
from sub_module import *
import global_ # Qui son salvate le variabili Globali
import time

# Questo starebbe dentro al server per mandare fuori la informazione
# Usando la funzione test_func()
while True:
    global_.num += 1
    # Cambia il valore delle PWM globali e mandale al blimp
    global_.pwm_left = 0.0
    global_.pwm_right = 0.0
    global_.pwm_up = 0.0
    pwm_func()
    time.sleep(1)