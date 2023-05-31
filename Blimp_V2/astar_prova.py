import warnings
import socket

from blimp_class import Madgwick, calibration, trilateration, TDoA, TDoA_dt, reject_outliers, ReadLine, PID_Controller, psi_map, Astar
import numpy as np
import math
from numpy.linalg import norm
from quaternion import Quaternion

import time
import select
import threading


import cv2


import timeit
smooth = True
if __name__ == "__main__":
    #img = cv2.flip(cv2.imread("C:\Volume_D\Programming\Blimp_git\Blimp\povo2_provaPathPlanning.png"), 0)
    img = cv2.flip(cv2.imread("/home/piblimp/Blimp_23/Blimp_V2/lab_meccatronica.pngC:\Volume_D\Programming\Blimp_git\Blimp\Blimp_V2\lab_meccatronica_V2.png"),0)
    #flip = img[::-1,:,:] # revise height in (height, width, channel)
    
    img[img > 128] = 255
    img[img <= 128] = 0
    m = np.asarray(img)
    m = cv2.cvtColor(m, cv2.COLOR_RGB2GRAY)
    m = m.astype(float) / 255.
    m = 1-cv2.dilate(1-m, np.ones((20, 20))) #inflate for avoid the obstacle 
    img = img.astype(float)/255.

    

    start = (int(1.50*100),150) ## MY case
    goal = (350, 800)

    

    a = timeit.default_timer()
    astar = Astar(m)
    path = astar.planning(start=start, goal=goal, img=img, inter=100) #10corrisponde a 1 metro qui
    path_array = np.array(path)/100.0
    print(path_array) # use path[0][0] per access specific indexon x ==> [0][i], y ==> [1][i] 
    b = timeit.default_timer()
    print("Time: ", b-a)
    
    print(path_array[0][0])

    

   