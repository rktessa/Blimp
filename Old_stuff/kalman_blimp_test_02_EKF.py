# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y directions,
# and the yaw orientation.
# Aug 23
# Riccardo Tessarin

import math
import numpy as np
from filterpy.common import Q_discrete_white_noise
from numpy.linalg import norm, inv
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


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
    c = 1.66/2.0 # m half lenght of blimp on x axis
    b = 0.553/2.0 # m half lenght of blimp on y axis radius
    dt = 1 
    xR = 0.0675 # m distance of R motor from CG
    xL = 0.0675 # m distance of L motor from CG
    m = 0.250 # kg total mass of airship
    I_z = m *(c*c + b*b)/5 # inertia
    Tk = 0.0
    deltaPhi = 0.0
    x_pos = 0.0
    y_pos = 0.0
    
    phi_pos = 0.0
    print(I_z)


    def __init__(self, dt=None, Tk= None, deltaPhi=None, x_pos=None, y_pos=None, phi_pos=None, pwm_l = None, pwm_r = None):
        """
        Initialize the class with the given parameters.
        :param dt: The sample period
        :return:
        """
        if dt is not None:
            self.dt = dt
        if Tk is not None:
            self.Tk = Tk
        if deltaPhi is not None:
            self.deltaPhi = deltaPhi
       
        if x_pos is not None:
            self.x_pos = x_pos
        if y_pos is not None:
            self.y_pos = y_pos
        
        if phi_pos is not None:
            self.phi_pos = phi_pos
        if pwm_l is not None:
            self.pwm_l = pwm_l
        if pwm_r is not None:
            self.pwm_r = pwm_r
    

    def input_control(self, pwm_l, pwm_r):
        '''Trasform the pwm command in a instaneous forward movement and angle rotation'''
        mpwm = 0.2
        self.Tk = (pwm_l/100 + pwm_r/100)/self.m  * self.dt**2 * mpwm   
        self.deltaPhi = (pwm_r/100 - pwm_l/100)/self.I_z * self.xR *self.dt**2 * mpwm *180 /np.pi
        
        
       
    def initialization(self):
        
        # State vector is x = [x, x'', y, y'', z, z'', phi, phi' ]
        # at the beginning is initialized with the value for position
        # measured by sonar, madgwick and uwbz
        x = np.transpose(np.array([self.x_pos, self.y_pos, self.phi_pos]))

        F = np.array([[1.0, 0.0, 0.0 ],
                    [0.0, 1.0, 0.0 ], 
                    [0.0, 0.0, 1.0 ]])

       
        # The force are rotated in the Blimp reference frame, to be considered with the IMU
        B = np.array([
                    [math.cos(self.phi_pos/180*np.pi), 0.0],
                    [math.sin(self.phi_pos/180*np.pi), 0.0],
                    [0.0, 1.0]])


        u_t =  np.transpose(np.array([self.Tk, self.deltaPhi])) #input vector depending on force

        V = np.array([[math.cos(self.phi_pos/180*np.pi), 0.0 ],
                    [math.sin(self.phi_pos/180*np.pi), 0.0 ], 
                    [0.0, 1.0 ]])
        
        grad_F = np.array([[1.0, 0.0, -self.Tk* math.sin(self.phi_pos/180*np.pi)],
                           [0.0, 1.0, self.Tk* math.cos(self.phi_pos/180*np.pi) ],
                           [0.0, 0.0, 1.0]])
        #State covariance matrix
        P = np.diag([15.0, 36.0, 360.0])
        # Q matrix found with error propagation of the control input equations
        M =  np.diag([2.45e-4, 5.58e-5*5])

        Q = V @ M @ np.transpose(V)

        # H is an Identity matrix beacuse all the quantities are directly measured by
        # a dedicated sensor
        H = np.diag([1., 1., 1.])

        R = np.diag([0.027, 0.0611, 2.12])
        
        return F, B, u_t ,x, P, Q, H, R, grad_F, M, V

    def predict_kal(self, x, P, F=1, Q=0, u=0, B=1, M= 0, grad_F = 0, V  = 0):
       

        if np.isscalar(F):
            F = np.array(F)

        # Devo Calcolare ad ogni step B,u_t, grad_f, V perchè sono funzioni di Tk, deltaPsi e angolo Psi
        phi = x[2]

        B = np.array([
                    [math.cos(phi/180*np.pi), 0.0],
                    [math.sin(phi/180*np.pi), 0.0],
                    [0.0, 1.0]])

        
        # u_t lo controllo dall'esterno con la funzione "control_input" 
        u_t =  np.transpose(np.array([self.Tk, self.deltaPhi])) #input vector depending on force'''

        V = np.array([[math.cos(phi/180*np.pi), 0.0 ],
                    [math.sin(phi/180*np.pi), 0.0 ], 
                    [0.0, 1.0 ]])
        
        grad_F = np.array([[1.0, 0.0, -self.Tk* math.sin(phi/180*np.pi)],
                           [0.0, 1.0, self.Tk* math.cos(phi/180*np.pi) ],
                           [0.0, 0.0, 1.0]])
       
        
        
        x = np.matmul(F, x) + np.matmul(B, u_t)
        
        P = grad_F @ P @ np.transpose(grad_F) + V @ M @ np.transpose(V)
        return x, P
    
    def update_kal(self, x, P, z, R, H=None, return_all=False):
       

        if z is None:
            if return_all:
                return x, P, None, None, None, None
            return x, P

        if H is None:
            H = np.diag([1., 1., 1. ])

        if np.isscalar(H):
            H = np.array([H])

        Hx = np.atleast_1d(np.matmul(H, x))
        z = reshape_z(z, Hx.shape[0], x.ndim)

        # error (residual) between measurement and prediction
        y = z - H@x

        # project system uncertainty into measurement space
        S = np.matmul(np.matmul(H, P), H.T) + R


        # map system uncertainty into kalman gain
        try:
            K = np.matmul(np.matmul(P, H.T), inv(S))
        except:
            # can't invert a 1D array, annoyingly
            K = np.matmul(np.matmul(P, H.T), 1./S)


        # predict new x with residual scaled by the kalman gain
        x = x + np.matmul(K, y)

        # P = (I-KH)P(I-KH)' + KRK'
        KH = np.matmul(K, H)

        try:
            I_KH = np.eye(KH.shape[0]) - KH
        except:
            I_KH = np.array([1 - KH])
        #P = np.matmul(np.matmul(I_KH, P), I_KH.T) + np.matmul(np.matmul(K, R), K.T)
        P = np.matmul(I_KH, P)

        if return_all:
            # compute log likelihood
            log_likelihood = np.logpdf(z, np.matmul(H, x), S)
            return x, P, y, K, S, log_likelihood
        return x, P

   
     
     
# Example of how use the code for the Kalman filter
if __name__ == "__main__":

    # Open the file with the experiment data
    
    with open('C:\Volume_D\Programming\Blimp_git\Blimp\Altitude control\prova_21_45.txt', encoding='utf-8') as file: 
    
        lines = file.readlines()

    mainlist = []   
    for pippo in range(len(lines)):   
        lines2 = lines[pippo].strip().split(' ')
        mainlist.append(lines2)

   
    time =   []
    x_pos = []
    y_pos = []
    yaw = []
    for row in range(len(mainlist)):
        time.append(float(mainlist[row][0])) 
        x_pos.append(float(mainlist[row][1])) 
        y_pos.append(float(mainlist[row][2]))
        yaw.append(float(mainlist[row][3]))
   
    # Open the file with the experiment nav data
   
    with open('C:\Volume_D\Programming\Blimp_git\Blimp\Altitude control\log_Blimp_navigation_2023-07-20 21-45-02.txt', encoding='utf-8') as file3:    
        lines3 = file3.readlines()

    mainlist = []   
    for pippo in range(len(lines3)):   
        lines4 = lines3[pippo].strip().split(' ')
        mainlist.append(lines4)

   
    time_nav = []
    l_pwm    = []
    r_pwm    = []
    psi_ref  = []
    
    for row in range(len(mainlist)):
        time_nav.append(float(mainlist[row][0])) 
        l_pwm.append(float(mainlist[row][1])) 
        r_pwm.append(float(mainlist[row][2]))
        psi_ref.append(float(mainlist[row][5]))


    for b in range(len(psi_ref)):
        if psi_ref[b] < -80:
            psi_ref[b] = psi_ref[b]+360
    # Initialization Kalman Blimp class
    kal = kalman_blimp(dt = 0.1, Tk=0.0,  deltaPhi = 0.0 , phi_pos= yaw[0],x_pos=x_pos[0], y_pos=y_pos[0] )
   
    # When the class is initialized, here I can insert the intial coordinate
    # for position and orientation in Global reference frame. 

    
    kal.input_control(pwm_l=l_pwm[0], pwm_r=r_pwm[0])
    print('r pwm = ' , r_pwm[0])
    F, B, u_t ,x, P, Q, H, R, grad_F, M, V = kal.initialization() 
    print('u = ', u_t)

    print('B = ', B)

    # Initialize the state vector for each phase
    x_pred = np.array([0., 0., 0.])  # sto usando solo questo alla fine
    
    # Initialize a counter for data 
    i = 0
    # counter for navigation data
    j = 0

    # Loop to simulate the evolution of the dynamics and
    # store the values for visualization
    while i < (len(time)-1): # finche non ho fatto tutti gli step
        # Calcolo il dt vero dell'esperimento
        kal.dt = time[i+1]- time[i]
        
        if time[i] > time_nav[j]: # per cambiare pwm al tempo giusto nella simulazione 
            if j < (len(time_nav)-1):
                j = j+1
                kal.input_control(pwm_l=l_pwm[j], pwm_r=r_pwm[j])
            

        x, P = kal.predict_kal(x=x, P=P, M = M, F=F )
        
        #attacco questo x trovato a x_pred
        pip = np.vstack((x_pred, x)) 
        x_pred = pip #rinomino il vettore
        
        

        #Measurement vector lo aggiorno a ogni step con il giusto valore misurato veramente
        
        z = np.array([[x_pos[i], y_pos[i], yaw[i]]]).T
    
        x, P = kal.update_kal(x, P, z, R, H)
        i+=1

    # rimuovo la prima riga perchè era di zeri
    x_pred = np.delete(x_pred, (0), axis=0) 


    # I valori trovati con il Kalman
    x_P_K = x_pred[:,0]
    y_P_K = x_pred[:,1]
    yaw_kalman = x_pred[:, 2]
    
    #The value of P at the end of the simulation
    print('P = ', P)

    
    # Figure Plot

    fig = plt.figure()
    ax = plt.axes()
    ax.scatter(x_pos[0], y_pos[0], label= 'Starting point', color='green')
    ax.plot(x_P_K, y_P_K, color = 'red', label= 'position estimated')
    ax.scatter(x_pos, y_pos, label= 'position measured with UWB', s=5, color= 'blue')
    ax.scatter(3, 2, label= 'Goal point', color='orange') 
    ax.axis('equal')
    ax.set_xlabel('X coordinate [m]')
    ax.set_ylabel('Y coordinate [m]')
    ax.set_title('Position during blimp motion')
    ax.legend()



    fig2 = plt.figure()
    ax2 = plt.axes()
    ax2.plot(time,yaw, label='Yaw measured')
    ax2.plot(time[0:-1],yaw_kalman,  color = 'red', label='Yaw Kalman')
    ax2.plot(time_nav, psi_ref,  color = 'green', label='Yaw goal')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Yaw angle [degrees]')
    ax2.set_title('Yaw angle during blimp motion')
    ax2.legend()


    plt.show()



