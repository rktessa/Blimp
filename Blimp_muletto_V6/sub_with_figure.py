import zmq
import sys 
import numpy as np
import time
import matplotlib.pyplot as plt
plt.rcParams["figure.figsize"] = 4,3
from matplotlib.animation import FuncAnimation
import logging
from blimp_class import *

# Versione 21 Aprile
# Specify port and IP adress for connections
port = "5556"


ip_address = "192.168.1.104"




def set_size(w,h, ax=None):
    """ w, h: width, height in inches """
    if not ax: ax=plt.gca()
    l = ax.figure.subplotpars.left
    r = ax.figure.subplotpars.right
    t = ax.figure.subplotpars.top
    b = ax.figure.subplotpars.bottom
    figw = float(w)/(r-l)
    figh = float(h)/(t-b)
    ax.figure.set_size_inches(figw, figh)


try:
    # Definizione context per zmq
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://192.168.1.104:5556")
    print("Mi sono connesso al Raspberry pi Blimp")

    # Subscribe to zipcode, default is Blimp
    zip_code = "Blimp"
    socket.setsockopt_string(zmq.SUBSCRIBE, zip_code)
except KeyboardInterrupt:
    print("Interrupted")
    logging.exception("message")


def str_to_float():
    try:
        message2 = socket.recv_string()
        zip_code, tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, mesg, zpos = message2.split(",")
        tempo = float(tempo)
        raw_accX = float(raw_accX)
        raw_accY = float(raw_accY)
        raw_accZ = float(raw_accZ)
        raw_gyrX = float(raw_gyrX)
        raw_gyrY = float(raw_gyrY)
        raw_gyrZ = float(raw_gyrZ)
        raw_magX = float(raw_magX)
        raw_magY = float(raw_magY)
        raw_magZ = float(raw_magZ)
        zpos = float(zpos)
        return tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg
    except KeyboardInterrupt:
        print("Interrupted")
        logging.exception("message")

psi_list = []
x_list = []
y_list = []
z_list = []

# per UWB
tempi_dt = np.zeros((1,6))


if __name__ == "__main__":
    tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
    ts = mesg0.split(" ")
    if (len(ts)!=25):
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg0 = str_to_float()
        ts = mesg0.split(" ")
    dt_uwb = TDoA_dt(ts)  # Calcola dt iniziale
    tempi_dt[0,:] = dt_uwb

    plt.ion()
    while True:
        
        time_zero_mad = tempo
        time_zero_meas = time.perf_counter()
        time_zero_UWB = time.perf_counter()

    

        # Misuro con UWB la posizione nel piano  frattempo 
        tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
        ts = mesg.split(" ")
        if (len(ts)!=25):
            tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ, zpos, mesg = str_to_float()
            ts = mesg.split(" ")

        time_current_UWB = time.perf_counter()
            #if (time_current_UWB - time_zero_UWB) >= 0.09:

        raw_acc = [raw_accX, raw_accY, raw_accZ]
        raw_gyr = [raw_gyrX, raw_gyrY, raw_gyrZ] 
        raw_mag = [raw_magX, raw_magY, raw_magZ]
        
        
        # Aquisizione magnetometro e calibrazione dei dati:
        mag = calibration(raw_mag)

        raw_acc, raw_gyr, mag = imu_to_uwb(raw_acc, raw_gyr, mag)

        x_pos_uwb, y_pos_uwb, z_pos_uwb, dt_new = TDoA(ts, dt_uwb)    

        if abs(x_pos_uwb) <= 25.0 and abs(y_pos_uwb) <= 25.0 and x_pos_uwb >= 0 and y_pos_uwb >= 0 :
                x_list.append(x_pos_uwb)
                y_list.append(y_pos_uwb)
                z_list.append(zpos)
                

        if len(x_list) > 10:
            x_list.pop(0)

        if len(y_list) > 10:
            y_list.pop(0)

        if len(z_list) > 10:
            z_list.pop(0)
        
        for i in range(6) :
            dt_uwb[i] = np.mean(reject_outliers(tempi_dt[:,i], m=2))

        x = x_pos_uwb
        y = y_pos_uwb
        print(x,y)

        
        fig, ax=plt.subplots()
        ax = plt.scatter(x,y)
        #set_size(5,5)

        plt.xlim([0, 5])
        plt.ylim([0, 11])  
        plt.title("Stanza")
        plt.draw()
        plt.pause(0.01)
        ax.remove()
        plt.close()
        

        plt.show(block=True) # block=True lets the window stay open at the end of the animation.
    

    '''
    # create a figure with an axes
    fig, ax = plt.subplots()
    # set the axes limits
    ax.axis([-0.5,5.0,-0.5,12.0])
    # set equal aspect such that the circle is not shown as ellipse
    #ax.set_aspect("equal")
    # create a point in the axes
    point, = ax.plot(0,1, marker="o")

    # Updating function, to be repeatedly called by the animation
    def update():
        # obtain point coordinates 
        #x,y = circle(phi)
        x_pos_a,y_pos_a = x,y
        # set point's coordinates
        point.set_data([x_pos_a],[y_pos_a])
        return point,

    # create animation with 10ms interval, which is repeated,
    # provide the full circle (0,2pi) as parameters
    ani = FuncAnimation(fig, update)

    plt.show()
    '''