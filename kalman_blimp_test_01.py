# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y, z directions,
# the acceleration in the same directions and the yaw orientation and angular velocity.
# Jan 23
# Riccardo Tessarin, Federico Marchi

import math
import numpy as np
from filterpy.common import Q_discrete_white_noise
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

#from filterpy.kalman import predict, update

## PARTE CHE SERVE SOLO PER I TEST 
#Definizione di tutti i pin per uso
'''import warnings
import socket
import RPi.GPIO as IO 
from blimp_class import Madgwick, calibration, orientation_initial
from numpy.linalg import norm
from quaternion import Quaternion
import time
import select
import threading
import board
from adafruit_icm20x import ICM20948, MagDataRate
from main_blimp import misuration

m1 = 20
HCtrig=23     
HCecho=24     

IO.setwarnings(False)           
IO.setmode(IO.BCM)        #numerazione bcm dei pin
IO.setup(m1,IO.OUT)       # setup motori     
IO.setup(HCtrig,IO.OUT)   # setup ultrasuoni
IO.setup(HCecho,IO.IN)  

p1 = IO.PWM(m1,100)       #inizializzazione pwm        
p1.start(0)                           
i2c = board.I2C()  # uses board.SCL and board.SDA
icm = adafruit_icm20x.ICM20948(i2c)

# Setup impostazioni IMU
icm.accelerometer_data_rate_divisor = 0  # Max velocity of sensor acquisition
icm.gyro_data_rate_divisor = 0 # Max velocity of sensor acquisition
icm.magnetometer_data_rate = MagDataRate.RATE_100HZ

## FINE''' 

def reshape_z(z, dim_z, ndim):
    """ ensure z is a (dim_z, 1) shaped vector"""

    z = np.atleast_2d(z)
    if z.shape[1] == dim_z:
        z = z.T

    if z.shape != (dim_z, 1):
        raise ValueError('z must be convertible to shape ({}, 1)'.format(dim_z))

    if ndim == 1:
        z = z[:, 0]

    if ndim == 0:
        z = z[0, 0]

    return z








# Definition of the matrices used in 
# Kalman filter
class kalman_blimp:
    c = 1.60/2.0 # m half lenght of blim on x axis
    b = 0.50/2.0 # m half lenght of blim on y axis
    dt = 0.01
    xR = 0.0675 # m distance of R motor from CG
    xL = 0.0675 # m distance of R motor from CG
    m = 0.2713 # kg total mass of airship
    I_z = m *(c*c + b*b)/5 # inertia
    Fl = 0.0 # N forces of the motors
    Fr = 0.0
    Fu = 0.0
    x_pos = 0.0
    y_pos = 0.0
    z_pos  = 0.0 
    phi_pos = 0.0


    def __init__(self, dt=None, Fl= None, Fr=None, Fu=None, x_pos=None, y_pos=None, z_pos=None, phi_pos=None):
        """
        Initialize the class with the given parameters.
        :param dt: The sample period
        :return:
        """
        if dt is not None:
            self.dt = dt
        if Fl is not None:
            self.Fl = Fl
        if Fr is not None:
            self.Fr = Fr
        if Fu is not None:
            self.Fu = Fu
        if x_pos is not None:
            self.x_pos = x_pos
        if y_pos is not None:
            self.y_pos = y_pos
        if x_pos is not None:
            self.z_pos = z_pos
        if x_pos is not None:
            self.phi_pos = phi_pos
       
    def initialization(self, phi):
        
        # State vector is x = [x, x', y, y', z, z', phi, phi' ]
        # at the beginning is initialized with the value for position
        # measured by sonar, madgwick and uwbz
        x = np.transpose(np.array([self.x_pos, 0.0, self.y_pos, 0.0, self.z_pos, 0.0, self.phi_pos, 0.0]))

        F = np.array([[1.0, (self.dt*self.dt)/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 
                    [0.0, 0.0, 1.0, (self.dt*self.dt)/2.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 0.0, 1.0, (self.dt*self.dt)/2.0, 0.0, 0.0 ],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 ], 
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, self.dt],
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

        B = np.array([[0.0, 0.0, 0.0],
                    [1.0/self.m, 1.0/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])

        

        u_t =  np.transpose(np.array([self.Fl, self.Fr, self.Fu])) #input vector depending on force

        
        
        #State covariance matrix
        P = np.diag([2.0, 0.1, 2.0, 0.1, 2.0, 0.1, 360.0, 10.0 ])
        
        # Q è calcolata a caso al momento
        Q =  np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ])
    
        H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])

        R = np.diag([0.01, 0.001, 0.01, 0.001, 0.01, 0.001, 0.001, 0.001])
        
        return F, B, u_t ,x, P, Q, H, R

    def predict_kal(self, x, P, F=1, Q=0, u=0, B=1, alpha=1.):
        """
        Predict next state (prior) using the Kalman filter state propagation
        equations.

        Parameters
        ----------

        x : numpy.array
            State estimate vector

        P : numpy.array
            Covariance matrix

        F : numpy.array()
            State Transition matrix

        Q : numpy.array, Optional
            Process noise matrix


        u : numpy.array, Optional, default 0.
            Control vector. If non-zero, it is multiplied by B
            to create the control input into the system.

        B : numpy.array, optional, default 0.
            Control transition matrix.

        alpha : float, Optional, default=1.0
            Fading memory setting. 1.0 gives the normal Kalman filter, and
            values slightly larger than 1.0 (such as 1.02) give a fading
            memory effect - previous measurements have less influence on the
            filter's estimates. This formulation of the Fading memory filter
            (there are many) is due to Dan Simon

        Returns
        -------

        x : numpy.array
            Prior state estimate vector

        P : numpy.array
            Prior covariance matrix
        """

        if np.isscalar(F):
            F = np.array(F)

        phi = x[6]
        R = np.array([[math.cos(phi), 0., - math.sin(phi), 0., 0., 0., 0., 0.],
                      [0., 1., 0, 0., 0., 0., 0., 0.,],
                      [math.sin(phi), 0, math.cos(phi), 0. , 0., 0., 0., 0.],
                      [0., 0., 0., 1., 0., 0., 0., 0.],
                      [0., 0., 0., 0., 1., 0., 0., 0.],
                      [0., 0., 0., 0., 0., 1., 0., 0.],
                      [0., 0., 0., 0., 0., 0., 1., 0.],
                      [0., 0., 0., 0., 0., 0., 0., 1.]])

        B2 = np.array([[0.0, 0.0, 0.0],
                    [math.cos(phi)/self.m, math.cos(phi)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [math.sin(phi)/self.m, math.sin(phi)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])
        
        #x_pre = np.dot(R,F)
        #x = np.dot(x_pre, x) + np.dot(B, u)
        x = np.dot(F, x) + np.dot(B2, u)
        P = (alpha * alpha) * np.dot(np.dot(F, P), F.T) + Q
        # write in a csv file the x at each iteration
        return x, P
    
    def update_kal(self, x, P, z, R, H=None, return_all=False):
        """Add a new measurement (z) to the Kalman filter. If z is None, nothing
        is changed.

        This can handle either the multidimensional or unidimensional case. If
        all parameters are floats instead of arrays the filter will still work,
        and return floats for x, P as the result.

        update(1, 2, 1, 1, 1)  # univariate
        update(x, P, 1



        Parameters
        ----------

        x : numpy.array(dim_x, 1), or float
            State estimate vector

        P : numpy.array(dim_x, dim_x), or float
            Covariance matrix

        z : (dim_z, 1): array_like
            measurement for this update. z can be a scalar if dim_z is 1,
            otherwise it must be convertible to a column vector.

        R : numpy.array(dim_z, dim_z), or float
            Measurement noise matrix

        H : numpy.array(dim_x, dim_x), or float, optional
            Measurement function. If not provided, a value of 1 is assumed.

        return_all : bool, default False
            If true, y, K, S, and log_likelihood are returned, otherwise
            only x and P are returned.

        Returns
        -------

        x : numpy.array
            Posterior state estimate vector

        P : numpy.array
            Posterior covariance matrix

        y : numpy.array or scalar
            Residua. Difference between measurement and state in measurement space

        K : numpy.array
            Kalman gain

        S : numpy.array
            System uncertainty in measurement space

        log_likelihood : float
            log likelihood of the measurement"""

        #pylint: disable=bare-except

        if z is None:
            if return_all:
                return x, P, None, None, None, None
            return x, P

        if H is None:
            H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])

        if np.isscalar(H):
            H = np.array([H])

        Hx = np.atleast_1d(np.dot(H, x))
        z = reshape_z(z, Hx.shape[0], x.ndim)

        # error (residual) between measurement and prediction
        y = z - Hx

        # project system uncertainty into measurement space
        S = np.dot(np.dot(H, P), H.T) + R


        # map system uncertainty into kalman gain
        try:
            K = np.dot(np.dot(P, H.T), inv(S))
        except:
            # can't invert a 1D array, annoyingly
            K = np.dot(np.dot(P, H.T), 1./S)


        # predict new x with residual scaled by the kalman gain
        x = x + np.dot(K, y)

        # P = (I-KH)P(I-KH)' + KRK'
        KH = np.dot(K, H)

        try:
            I_KH = np.eye(KH.shape[0]) - KH
        except:
            I_KH = np.array([1 - KH])
        P = np.dot(np.dot(I_KH, P), I_KH.T) + np.dot(np.dot(K, R), K.T)


        if return_all:
            # compute log likelihood
            log_likelihood = np.logpdf(z, np.dot(H, x), S)
            return x, P, y, K, S, log_likelihood
        return x, P

    def sim_blimp(self, x):
        # x is the array found with predict method

        '''At each step the acceleration will vary according
        to the process variance process_var. After updating
        the position we compute a measurement with an assumed
        sensor variance of z_var.'''
        proc_var = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ]) #correspond to Q
        z_var = np.array([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]) # correspond to R
        z_std = np.sqrt(z_var) 
        p_std = np.sqrt(proc_var)
        pos_x = x[0] + np.random.randn() * z_std[0]
        acc_x = x[1] + np.random.randn() * z_std[1]
        pos_y = x[2] + np.random.randn() * z_std[2]
        acc_y = x[3] + np.random.randn() * z_std[3]
        pos_z = x[4] + np.random.randn() * z_std[4]
        acc_z = x[5] + np.random.randn() * z_std[5]
        phi_m= x[6] + np.random.randn() * z_std[6]
        phi_vel = x[7] + np.random.randn() * z_std[7]
#DA CAPIRE COME FINIRE
        '''v = vel + (randn() * p_std)
        x += v*dt        
        xs.append(x)
        zs.append(x + randn() * z_std)'''

        # Da scrivere per simulare kalman 
        return pos_x, acc_x, pos_y, acc_y, pos_z, acc_z, phi_m, phi_vel




        
     
# Example of how use the code for the Kalman filter
if __name__ == "__main__":

    kal = kalman_blimp(dt = 0.1, Fl = 0.2,  Fr = 0.2 , Fu = 0.01)
    i = 0
    phi = 0.0
    F, B, u_t ,x, P , Q, H, R = kal.initialization(phi) 

    x_pred = np.array([0., 0., 0., 0., 0., 0., 0., 0.])
    x_up = np.array([0., 0., 0., 0., 0., 0., 0., 0.]).T
    x_mes = np.array([0., 0., 0., 0., 0., 0., 0., 0.]).T
    

while i<100: 
    
    x, P = kal.predict_kal(x=x, P=P, F=F, Q=Q, u=u_t, B=B)
    

    pip = np.vstack((x_pred, x))
    x_pred = pip
    #print(x)
    # Intanto per i test le misure le genero io con funzione kal.sim_blimp()
    #pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel = misuration()

    #Measurement vector
    pos_x, acc_x, pos_y, acc_y, pos_z, acc_z, phi_m, phi_vel = kal.sim_blimp(x)
    z = np.array([[pos_x, acc_x, pos_y, acc_y, pos_z, acc_z, phi_m, phi_vel]]).T
   
    x, P = kal.update_kal(x, P, z, R, H)
    i+=1
x_pred = np.delete(x_pred, (0), axis=0)

x_P = x_pred[:,0]
y_P = x_pred[:,2]
z_P = x_pred[:, 4]
phi_mis = (x_pred[:, 6])/180*np.pi
time = np.arange(0, 100, 1)
print(x_P)

fig1 =plt.figure()
plt.plot(x_P,y_P)
plt.grid()

fig2 = plt.figure()
plt.plot(time,phi_mis)
plt.grid()

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x_P, y_P, z_P, 'red')
ax.scatter3D(x_P, y_P, z_P, c=z_P, cmap='Greens');


plt.show()



