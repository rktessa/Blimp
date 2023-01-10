# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y, z directions,
# the acceleration in the same directions and the yaw orientation and angular velocity.
# Jan 23
# Riccardo Tessarin, Federico Marchi

import math
import numpy as np
from filterpy.common import Q_discrete_white_noise
from blimp_class import misuration
#from filterpy.kalman import predict, update

# Definition of the matrices used in 
# Kalman filter
class kalman_blimp:
    c = 1.60/2.0 # m half lenght of blim on x axis
    b = 0.50/2.0 # m half lenght of blim on y axis
    dt = 0.01
    xR = 0.05 # m distance of R motor from CG
    xL = 0.05 # m distance of R motor from CG
    m = 0.25 # kg total mass of airship
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
        x = np.array([[self.x_pos, 0.0, self.y_pos, 0.0, self.z_pos, 0.0, self.phi_pos, 0.0]]).T

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

        u_t = np.array([[self.Fl, self.Fr, self.Fu]]).T #input vector depending on force

        
        
        #State covariance matrix
        P = np.diag([2.0, 0.1, 2.0, 0.1, 2.0, 0.1, 360.0, 10.0 ])
        
        # Q è calcolata a caso al momento
        Q =  np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01 ])
    
        H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])
        
        return F, B, u_t ,x, P, Q, H

    def predict(self, x, P, F=1, Q=0, u=0, B=1, alpha=1.):
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
        x_pre = np.dot(R,F)
        x = np.dot(x_pre, x) + np.dot(B, u)
        P = (alpha * alpha) * np.dot(np.dot(F, P), F.T) + Q
        # write in a csv file the x at each iteration
        return x, P
    


        
     
# Example of how use the code for the Kalman filter
   
kal = kalman_blimp(dt = 0.1, Fl = 2.0, Fr = 5.9, Fu = 2.0)
i = 0
phi = 0.0
F, B, u_t ,x, P , Q, H= kal.initialization(phi) 

pos_x, pos_y, pos_z, acc_x, acc_y, acc_z, phi_vel = misuration()

#Measuraement vector
z = np.array([[pos_x, acc_x, pos_y, acc_y, pos_z, acc_z, phi_m, phi_vel]]).T

while i<10: 
    # Q is the process noise x, P, F=1, Q=0, u=0, B=1, alpha=1.
    
    x, P = kal.predict(x=x, P=P, F=F, Q=Q, u=u_t, B=B)
   
    print('P =', P)
    i+=1
