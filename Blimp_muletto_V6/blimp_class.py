
import warnings
import time
import numpy as np
from numpy.linalg import norm
from quaternion import Quaternion
import pandas as pd
from matplotlib import pyplot as plt
import math
from math import dist, cos, sin
import cv2




# FUNCTIONS AND CLASSES USED FOR ESTIMATE THE BLIMP ORIENTATION IN SPACE

# Definition of the matrices used in 
# Kalman filter
class kalman_blimp:
    c = 1.60/2.0 # m half lenght of blimp on x axis
    b = 0.40/2.0 # m half lenght of blimp on y axis
    dt = 1 
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
       
    def initialization(self):
        
        # State vector is x = [x, x'', y, y'', z, z'', phi, phi' ]
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

        '''B = np.array([[0.0, 0.0, 0.0],
                    [1.0/self.m, 1.0/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])'''
        B = np.array([[0.0, 0.0, 0.0],
                    [math.cos(self.phi_pos)/self.m, math.cos(self.phi_pos)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [math.sin(self.phi_pos)/self.m, math.sin(self.phi_pos)/self.m, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0/self.m],
                    [0.0, 0.0, 0.0],
                    [self.xL*self.dt/self.I_z, -self.xR*self.dt/self.I_z, 0.0]])

        

        u_t =  np.transpose(np.array([self.Fl, self.Fr, self.Fu])) #input vector depending on force

        
        
        #State covariance matrix
        # maybe in P it is better to put a rad and rad/s value for phi and phi vel.
        P = np.diag([0.0064, 0.0005, 0.0030, 0.0007, 0.00034, 0.002, 0.00015, 0.0000012])
        # Q calcolata a caso al momento
        Q =  np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 ])

        #Q = Q_discrete_white_noise(dim=2, dt=0.5, var=1, block_size=4)
        #Q = block_diag(q, q)
        print(Q)

        # H is an Identity matrix beacuse all the quantities are directly measured by
        # a dedicated sensor
        H = np.diag([1., 1., 1., 1., 1., 1., 1., 1. ])

        R = np.diag([0.0575, 0.0045, 0.0267, 0.0064, 0.0027, 0.0176, 0.0013, 0.000011])
        #R = np.diag([0.09, 0.0005, 0.09, 0.0007, 0.09, 0.002, 0.00015, 0.0000012])
        
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
        
        x = np.matmul(F, x) + np.matmul(B2, u)
        P = (alpha * alpha) * np.matmul(np.matmul(F, P), F.T) + Q
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

        Hx = np.atleast_1d(np.matmul(H, x))
        z = reshape_z(z, Hx.shape[0], x.ndim)

        # error (residual) between measurement and prediction
        y = z - Hx

        # project system uncertainty into measurement space
        S = np.matmul(np.matmul(H, P), H.T) + R
        #print(S)
        #print(1./S)
        #print(inv(S))


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




# Conversion from rad to deg for visualization on terminal
def rad_to_deg(roll, pitch, yaw):

    roll_deg = roll * 180/np.pi
    pitch_deg =pitch * 180/np.pi
    yaw_deg = yaw * 180/np.pi 

    if yaw_deg < 0:
        yaw_deg +=  360
    if roll_deg <0:
        roll_deg+= 360
    if pitch_deg < 0:
        pitch_deg += 360 
    return roll_deg, pitch_deg, yaw_deg

class Madgwick:
    samplePeriod = 1/100
    quaternion = Quaternion(0, 0, 1, 0)
    beta = 0.35 # Questo � il nuovo beta usato per far funzionare meglio il Madgwick. valori di stabilit� oscillano sui 3 gradi 
    zeta = 0

    def __init__(self, sampleperiod=None, quaternion=None, beta=None, zeta=None):
        """
        Initialize the class with the given parameters.
        :param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :param beta: Algorithm gain zeta
        :return:
        """
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if zeta is not None:
            self.zeta = zeta

    def update(self, gyroscope, accelerometer, magnetometer):
        """
        Perform one update step with data from a AHRS sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        :param magnetometer: A three-element array containing the magnetometer data. Can be any unit since a normalized value is used.
        :return:
        """
        q = self.quaternion
        #print(self.samplePeriod)
        gyroscope = np.array(gyroscope*np.pi/180, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()
        magnetometer = np.array(magnetometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) == 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Normalise magnetometer measurement
        if norm(magnetometer) == 0:
            warnings.warn("magnetometer is zero")
            return
        magnetometer /= norm(magnetometer)

        h = q * (Quaternion(0, magnetometer[0], magnetometer[1], magnetometer[2]) * q.conj())
        b = np.array([0, norm(h[1:3]), 0, h[3]])

        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2]) - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2],
            2*b[1]*(0.5 - q[2]**2 - q[3]**2) + 2*b[3]*(q[1]*q[3] - q[0]*q[2]) - magnetometer[0],
            2*b[1]*(q[1]*q[2] - q[0]*q[3]) + 2*b[3]*(q[0]*q[1] + q[2]*q[3]) - magnetometer[1],
            2*b[1]*(q[0]*q[2] + q[1]*q[3]) + 2*b[3]*(0.5 - q[1]**2 - q[2]**2) - magnetometer[2]
        ])
        j = np.array([
            [-2*q[2],                  2*q[3],                  -2*q[0],                  2*q[1]],
            [2*q[1],                   2*q[0],                  2*q[3],                   2*q[2]],
            [0,                        -4*q[1],                 -4*q[2],                  0],
            [-2*b[3]*q[2],             2*b[3]*q[3],             -4*b[1]*q[2]-2*b[3]*q[0], -4*b[1]*q[3]+2*b[3]*q[1]],
            [-2*b[1]*q[3]+2*b[3]*q[1], 2*b[1]*q[2]+2*b[3]*q[0], 2*b[1]*q[1]+2*b[3]*q[3],  -2*b[1]*q[0]+2*b[3]*q[2]],
            [2*b[1]*q[2],              2*b[1]*q[3]-4*b[3]*q[1], 2*b[1]*q[0]-4*b[3]*q[2],  2*b[1]*q[1]]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Gyroscope compensation drift
        gyroscopeQuat = Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])
        stepQuat = Quaternion(step.T[0], step.T[1], step.T[2], step.T[3])

        gyroscopeQuat = gyroscopeQuat + (q.conj() * stepQuat) * 2 * self.samplePeriod * self.zeta * -1

        # Compute rate of change of quaternion
        qdot = (q * gyroscopeQuat) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = Quaternion(q / norm(q))  # normalise quaternion
        #quat = Quaternion(q / norm(q))
        return self.quaternion

    def __str__ (self):
        return  str(self.x)


    
def calibration(magnetic):
    """
    This function is used to calibrate the magnetometer data from Hard
    and soft iron offsets. 
    """
    magnetic = np.array(magnetic, dtype=float).flatten() #Convert the measure in a numpy array
    
    df = pd.read_csv(r'C:\Volume_D\Programming\Blimp_git\Blimp\Blimp_muletto_V6\data.csv')
    
    # Hard Iron Vector
    b = np.array([df.iloc[0,1], df.iloc[0,2], df.iloc[0,3]])
    #Soft iron transformation matrix:
    A_1 = np.array([[df.iloc[1,1], df.iloc[1,2], df.iloc[1,3]],
                [df.iloc[2,1], df.iloc[2,2], df.iloc[2,3]],
                [df.iloc[3,1], df.iloc[3,2], df.iloc[3,3]]])

    mag=[0,0,0]
    
    #Hard iron bias:
    xm_off = magnetic[0]-b[0]
    ym_off = magnetic[1]-b[1]
    zm_off = magnetic[2]-b[2]


    #multiply by the inverse soft iron offset
    mag[0] = (xm_off *  A_1[0,0] + ym_off *  A_1[0,1]  + zm_off *  A_1[0,2])
    mag[1] = (xm_off *  A_1[1,0] + ym_off *  A_1[1,1]  + zm_off *  A_1[1,2])
    mag[2] = (xm_off *  A_1[2,0] + ym_off *  A_1[2,1]  + zm_off *  A_1[2,2])
    
    return mag



def print_calibration():
    
    data = np.loadtxt('data_for_calibration.txt',delimiter=',') #input


    plt.rcParams["figure.autolayout"] = True
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], marker='o', color='r')


    

    df = pd.read_csv('data.csv')
    
    # Hard Iron Vector
    b = np.array([df.iloc[0,1], df.iloc[0,2], df.iloc[0,3]])
    #Soft iron transformation matrix:
    A_1 = np.array([[df.iloc[1,1], df.iloc[1,2], df.iloc[1,3]],
                [df.iloc[2,1], df.iloc[2,2], df.iloc[2,3]],
                [df.iloc[3,1], df.iloc[3,2], df.iloc[3,3]]])

    print("Soft iron transformation matrix:\n", A_1)
    print("Hard iron bias:\n", b)


    result = [] 
    for row in data: 
    
        # subtract the hard iron offset
        xm_off = row[0]-b[0]
        ym_off  = row[1]-b[1]
        zm_off  = row[2]-b[2]
        

        #multiply by the inverse soft iron offset
        xm_cal = xm_off *  A_1[0,0] + ym_off *  A_1[0,1]  + zm_off *  A_1[0,2] 
        ym_cal = xm_off *  A_1[1,0] + ym_off *  A_1[1,1]  + zm_off *  A_1[1,2] 
        zm_cal = xm_off *  A_1[2,0] + ym_off *  A_1[2,1]  + zm_off *  A_1[2,2] 

        result = np.append(result, np.array([xm_cal, ym_cal, zm_cal]) )#, axis=0 )
        #result_hard_iron_bias = np.append(result, np.array([xm_off, ym_off, zm_off]) )

    result = result.reshape(-1, 3)
    
    ax.scatter(result[:,0], result[:,1], result[:,2], marker='o', color='g')
    plt.show()


def trilateration(d1,d2,d3,d4,d5,d6):

    # Anchor definition, set their position in space

    A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6  = np.array([3.00, 9.35, 3.15])

    An =  A_n1, A_n2, A_n3, A_n4, A_n5, A_n6
    An = np.array(An)  

    d = np.array([ d1, d2, d3, d4, d5, d6])

    # Compute the position of T using trilateration and LS formula
    # Defining initial A matrix and b vector
    A = np.zeros((5,3))
    b = np.zeros((5,1))

    # Definition of vectors X, Y, Z, first position is the target,
    # the others the anchors coordinates
    x = An[:,0]
    y = An[:,1]
    z = An[:,2]

    # Calculation of A and b for the case with 6 anchors
    for c in range(1, len(x)):
        A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0]] 
        #A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0], 0, 0, 0]  
        b[c-1] = d[0]**2 - d[c]**2 + x[c]**2 + y[c]**2 + z[c]**2 - x[0]**2 -y[0]**2 -z[0]**2   
        
    pos = np.matmul(np.linalg.pinv(A), b)/2
    return pos[0], pos[1], pos[2] #pos x,y,z   


def reject_outliers(data, m = 2.):
    d = np.abs(data - np.median(data))
    mdev = np.median(d)
    s = d/mdev if mdev else np.zeros(len(d))
    return data[s<m]



def TDoA_dt(ts):
    t6_rx1 = float(int( ts[0], 2 )) * 15.65e-12
    t1_rx1 = float(int( ts[1],2  )) * 15.65e-12
    t2_rx1 = float(int(ts[2],2)) * 15.65e-12
    t3_rx1 = float(int(ts[3],2)) * 15.65e-12
    t4_rx1 = float(int(ts[4],2)) * 15.65e-12
    t5_rx1 = float(int(ts[5],2)) * 15.65e-12

    t6_rx2 = float(int(ts[6],2)) * 15.65e-12
    t1_rx2 = float(int(ts[7],2)) * 15.65e-12
    t2_rx2 = float(int(ts[8],2)) * 15.65e-12
    t3_rx2 = float(int(ts[9],2)) * 15.65e-12
    t4_rx2 = float(int(ts[10],2)) * 15.65e-12
    t5_rx2 = float(int(ts[11],2)) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(int(ts[12],2)) * 15.65e-12
    t1_tx1 = float(int(ts[13],2)) * 15.65e-12
    t2_tx1 = float(int(ts[14],2)) * 15.65e-12
    t3_tx1 = float(int(ts[15],2)) * 15.65e-12
    t4_tx1 = float(int(ts[16],2)) * 15.65e-12
    t5_tx1 = float(int(ts[17],2)) * 15.65e-12

    t6_tx2 = float(int(ts[18],2)) * 15.65e-12
    t1_tx2 = float(int(ts[19],2)) * 15.65e-12
    t2_tx2 = float(int(ts[20],2)) * 15.65e-12
    t3_tx2 = float(int(ts[21],2)) * 15.65e-12
    t4_tx2 = float(int(ts[22],2)) * 15.65e-12
    t5_tx2 = float(int(ts[23],2)) * 15.65e-12
    
    # Embedded Lab system anchor position
    '''A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6 = np.array([3.00, 9.35, 3.15])'''

    A_n1 = [0.00, 7.19, 2.15]
    A_n2 = [0.00, 3.62, 3.15]
    A_n3 = [0.00, 0.00, 2.15]
    A_n4 = [4.79, 1.85, 3.15]
    A_n5 = [4.79, 5.45, 2.15]
    A_n6 = [3.00, 9.35, 3.15]

    A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
    c = 299792458 # Speed of light
    n = len(A_n)

    
    #TOF_MA = np.sqrt(np.sum)

    # Real measurements
    toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1, t1_tx2], [t2_tx1,t2_tx2], [t3_tx1,t3_tx2], [t4_tx1,t4_tx2], [t5_tx1,t5_tx2]])
    toa_rx = np.array([[t6_rx1,t6_rx2], [t1_rx1,t1_rx2], [t2_rx1,t2_rx2], [t3_rx1,t3_rx2], [t4_rx1,t4_rx2], [t5_rx1,t5_rx2]])

    #Drift tag
    dt_new = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])

    return dt_new



def TDoA(ts, dt):
    # Time definition
    '''t6_rx1 = float(ts[0,0]) * 15.65e-12
    t1_rx1 = float(ts[1,0]) * 15.65e-12
    t2_rx1 = float(ts[2,0]) * 15.65e-12
    t3_rx1 = float(ts[3,0]) * 15.65e-12
    t4_rx1 = float(ts[4,0]) * 15.65e-12
    t5_rx1 = float(ts[5,0]) * 15.65e-12

    t6_rx2 = float(ts[0,1]) * 15.65e-12
    t1_rx2 = float(ts[1,1]) * 15.65e-12
    t2_rx2 = float(ts[2,1]) * 15.65e-12
    t3_rx2 = float(ts[3,1]) * 15.65e-12
    t4_rx2 = float(ts[4,1]) * 15.65e-12
    t5_rx2 = float(ts[5,1]) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(ts[0,2]) * 15.65e-12
    t1_tx1 = float(ts[1,2]) * 15.65e-12
    t2_tx1 = float(ts[2,2]) * 15.65e-12
    t3_tx1 = float(ts[3,2]) * 15.65e-12
    t4_tx1 = float(ts[4,2]) * 15.65e-12
    t5_tx1 = float(ts[5,2]) * 15.65e-12

    t6_tx2 = float(ts[0,3]) * 15.65e-12
    t1_tx2 = float(ts[1,3]) * 15.65e-12
    t2_tx2 = float(ts[2,3]) * 15.65e-12
    t3_tx2 = float(ts[3,3]) * 15.65e-12
    t4_tx2 = float(ts[4,3]) * 15.65e-12
    t5_tx2 = float(ts[5,3]) * 15.65e-12
    '''
    
    t6_rx1 = float(int(ts[0],2)) * 15.65e-12
    t1_rx1 = float(int(ts[1],2)) * 15.65e-12
    t2_rx1 = float(int(ts[2],2)) * 15.65e-12
    t3_rx1 = float(int(ts[3],2)) * 15.65e-12
    t4_rx1 = float(int(ts[4],2)) * 15.65e-12
    t5_rx1 = float(int(ts[5],2)) * 15.65e-12

    t6_rx2 = float(int(ts[6],2)) * 15.65e-12
    t1_rx2 = float(int(ts[7],2)) * 15.65e-12
    t2_rx2 = float(int(ts[8],2)) * 15.65e-12
    t3_rx2 = float(int(ts[9],2)) * 15.65e-12
    t4_rx2 = float(int(ts[10],2)) * 15.65e-12
    t5_rx2 = float(int(ts[11],2)) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(int(ts[12],2)) * 15.65e-12
    t1_tx1 = float(int(ts[13],2)) * 15.65e-12
    t2_tx1 = float(int(ts[14],2)) * 15.65e-12
    t3_tx1 = float(int(ts[15],2)) * 15.65e-12
    t4_tx1 = float(int(ts[16],2)) * 15.65e-12
    t5_tx1 = float(int(ts[17],2)) * 15.65e-12

    t6_tx2 = float(int(ts[18],2)) * 15.65e-12
    t1_tx2 = float(int(ts[19],2)) * 15.65e-12
    t2_tx2 = float(int(ts[20],2)) * 15.65e-12
    t3_tx2 = float(int(ts[21],2)) * 15.65e-12
    t4_tx2 = float(int(ts[22],2)) * 15.65e-12
    t5_tx2 = float(int(ts[23],2)) * 15.65e-12
    
    # Embedded Lab system anchor position
    '''A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6 = np.array([3.00, 9.35, 3.15])'''

    A_n1 = [0.00, 7.19, 2.15]
    A_n2 = [0.00, 3.62, 3.15]
    A_n3 = [0.00, 0.00, 2.15]
    A_n4 = [4.79, 1.85, 3.15]
    A_n5 = [4.79, 5.45, 2.15]
    A_n6 = [3.00, 9.35, 3.15]

    A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
    c = 299792458 # Speed of light
    n = len(A_n)

    
    #TOF_MA = np.sqrt(np.sum)

    # Real measurements
    toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1, t1_tx2], [t2_tx1,t2_tx2], [t3_tx1,t3_tx2], [t4_tx1,t4_tx2], [t5_tx1,t5_tx2]])
    toa_rx = np.array([[t6_rx1,t6_rx2], [t1_rx1,t1_rx2], [t2_rx1,t2_rx2], [t3_rx1,t3_rx2], [t4_rx1,t4_rx2], [t5_rx1,t5_rx2]])

    #Drift tag
    dt_new = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])
    
    
    tmp_rx = np.zeros((len(toa_rx),2))
    tmp_rx[:,0] = toa_rx[: , 0] - toa_rx[0, 0] - (toa_tx[: ,0] * dt - toa_tx[0, 0]*dt[0])
    tmp_rx[:,1] = toa_rx[: , 1] - toa_rx[0, 1] - (toa_tx[:, 1] * dt - toa_tx[0, 1]*dt[0])
    
    ## TDoA
    #     tdoa = tmp_rx(:,2) - tmp_tx(:,2);
    tdoa = np.zeros((len(tmp_rx), 2))
    tdoa = tmp_rx[:, 1]
    tdoa = np.delete(tdoa, [0])

    D = c*tdoa
    
    #------Trilateration linear equations system-------------------
    A = np.array((A_n6[0] - A_n[1:n, 0], A_n6[1]-A_n[1:n, 1], A_n6[2]-A_n[1:n, 2], D)).T *2
    
    b = D**2 + np.linalg.norm(A_n6)**2 - np.sum(np.square(A_n[1:n, :]), axis=1)
    
   
    
    x_t0 = np.matmul(np.linalg.pinv(A), b.T)
    
    #print("x_t0 =", x_t0) 
    #print("A =", A)
    #print("pinvA = ", sp.linalg.pinv(A))
    #-----Non linear correction (Taylor Expansion)-----------------
    x_t_0 = np.array((x_t0[0], x_t0[1], x_t0[2]))
    f = np.zeros((n-1,1))
    del_f = np.zeros((n-1,3))
    #ii = 1
    
    
    
    for ii in range(1,n) :
        f[ii-1]= np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)-np.linalg.norm((x_t_0 - A_n[0,:]), ord = 2)
        del_f[ii-1,0] = (x_t_0[0] - A_n[ii,0])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[0]-A_n[0,0])*np.linalg.norm((x_t_0-A_n[0,:]), ord = 2)**-1
        del_f[ii-1,1] = (x_t_0[1] - A_n[ii,1])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[1]-A_n[0,1])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1
        del_f[ii-1,2] = (x_t_0[2] - A_n[ii,2])*np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)**-1 - (x_t_0[2]-A_n[0,2])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1 

    #print("res = ", (D-f.T) )
    apinv = np.linalg.pinv(del_f)
    abho = (D- f.T).T
    
    x_t = (np.matmul(np.linalg.pinv(del_f), (D- f.T).T)).T + x_t_0

    return x_t[0,0], x_t[0,1], x_t[0,2], dt_new  # Cos� abbiamo in input y, x, z (?) 



class Astar():
    def __init__(self, m): #m is the only input require while definig class, it is the map of the environment
        self.map = m
        self.initialize()

    def initialize(self): #It is necessary when I want to recalculate a new path from 2 points on the map
        self.open = []
        self.close = {}
        self.g = {} #distance from start to the node
        self.h = {} #distance from node to the goal
        node_goal = None

    # estimation
    def distance(self, a, b):
        # Diagonal distance
        dMax = np.max([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        dMin = np.min([np.abs(a[0]-b[0]), np.abs(a[1]-b[1])])
        d = math.sqrt(2.) * dMin + (dMax - dMin)
        return d

    def planning(self, start, goal, inter, img=None):
        self.initialize() # Initialize the calculator
        self.open.append(start) #first step of the alghorithm is put node start in open list
        self.close[start] = None
        self.g[start] = 0 # coordinate of start are at distance 0 from itself
        self.h[start] = self.distance(start, goal) # distance from start point to goal
        goal_node = None
        
        while(1): #Here start the real alghorithm
            min_distance = 9999999. #this value mast be maxed at the beginning
            min_id = -1 #not exist in reality this index
            for i, node in reversed(list(enumerate(self.open))): #mia modifica, prendo prima gli ultimi perch� sono tendenzialmente quelli con h minore
            #for i, node in enumerate(self.open): # controllo tutti i nodi in open
                # 1:(loc1, loc2) come sto chiamando con enumerate i vari nodi in open
                f = self.g[node] # Calcolo il costo in termini di distanza di tutti
                y = self.h[node] # i vicini del nodo che sto considerando

                if f+y < min_distance: # Alla fine tengo il nodo che minimizza la somma dei due valori
                    min_distance = f+y
                    min_id = i

            # Alla fine questo nodo migliore � quello che mi rimane e su cui faccio i calcoli
            p = self.open.pop(min_id)  # Tolgo quello che vado a considerare da open e tengo per� le sue coordinate

            #Controllo se in p vi � un ostacolo
            if self.map[p[1], p[0]] < 0.5: #Qui il check si basa sul colore della mappa, nero = ostacolo
                continue # In questo modo il punto con ostacolo viene tolto da open e non si sviluppano i conti su di esso.

            # Controllo se sono arrivato alla destinazione
            if self.distance(p, goal) < inter:
                self.goal_node = p # Salvo p per non perderlo in goal node
                break

            # eight directions, definisco le 8 direzioni cui posso muovermi dal punto in cui sono (loc1, loc2)
            pts_next1 = [(p[0]+inter, p[1]), (p[0], p[1]+inter),
                         (p[0]-inter, p[1]), (p[0], p[1]-inter)]
            pts_next2 = [(p[0]+inter, p[1]+inter), (p[0]-inter, p[1]+inter),
                         (p[0]-inter, p[1]-inter), (p[0]+inter, p[1]-inter)]
            pts_next = pts_next1 + pts_next2

            for pn in pts_next: #Itero sui punti
                # Ci sono 2 possibilit�, il punto � nuovo, oppure � gi� stato considerato
                # per i puntinuovi
                if pn not in self.close:
                    # metto pn in open
                    self.open.append(pn)
                    self.close[pn] = p # Associo pn al punto cui lo sto collegando
                    # Calcolo per quel punto il valore di h e g
                    self.g[pn] = self.g[p] + inter #La distanza da start che ha
                    self.h[pn] = self.distance(pn, goal) #quanto  distante ancora da goal
                
                # Se � gia in close devo controllare se la distanza g diminuisce
                elif self.g[pn] > self.g[p] + inter:
                    # In caso affermativo sostituisco con la distanza minore e cambio il parente
                    self.close[pn] = p
                    self.g[pn] = self.g[p] + inter 
                    # Non serve per� ricalcolare la h, quella non cambia

            '''if img is not None:
                cv2.circle(img, (start[0], start[1]), 5, (0, 0, 1), 3)
                cv2.circle(img, (goal[0], goal[1]), 5, (0, 1, 0), 3)
                cv2.circle(img, p, 2, (0, 0, 1), 1)
                img_ = cv2.flip(img, 0)
                cv2.imshow("A* Test", img_)
                k = cv2.waitKey(1)
                if k == 27:
                    break'''


        # Extract path
        path = []
        p = self.goal_node
        while(True):
            path.insert(0, p)
            if self.close[p] == None:
                break
            p = self.close[p] # Il valore precedente in close, chiama il suo parente che diventa il punto successivo del path e ricorsivamente ricostruisco il percorso
        if path[-1] != goal:
            path.append(goal)
        return path


#################################################################################
# CODICE PER CONVERTIRE GLI PSI DEL MADGWICK IN UNA ROTAZIONE DI 90 GRADI ESATTI 
#################################################################################
def psi_map(psi):

    N = 70
    E = 160
    S = 250
    W = 340

    E_new = N+90
    S_new = N+180
    W_new = N+270

    if N<=psi<E:
        psi_mapped = 90/(E-N)*(psi-N)+N
    elif E<=psi<S:
        psi_mapped = 90/(S-E)*(psi-E)+E_new
    elif S<=psi<W:
        psi_mapped = 90/(W-S)*(psi-S)+S_new
    elif W<=psi<360:
        psi_mapped = 90/(360-W+N)*(psi-W)+W_new
    elif 0<=psi<N:
            psi_mapped = 90/(360-W+N)*(psi)+0
    
    return psi_mapped

# PID FOR THE AUTONOMOUS NAVIGATION

# Create object class for PID_Controller. Object contains the PID cofficients, a method for calculating
# output signal based on system state, a method for running a system simulation based on the controller signal
# and a method for autotuning the attributes to optimize controller performance for an input system. 
class PID_Controller:
    
    # Initialize controller attributes when object is created. kp, ki, and kd are the constants for the
    # proportional, integral, and derivative elements of the controller, respectively. Target is an input attribute
    # which establishes the goal the controller is trying to achieve. Target is not set at object intiliazation, but
    # is rather set as part of "running" the controller. Signal is effectively an output attribute which represents 
    # the "current" signal being emitted from the controller. Accumulator is a attribute used to execute the integral 
    # calculation, and last_reading is using to determine the slope for application of the derivative calculation.
    # Max_signal is used to limit the signal request that the controller can make, and the sample_rate establishes how
    # frequently the controller can adjust the signal. In a practical application this would equate to a 
    # maximum speed/torque/force of a motor or actuator element and the sampling rate of a digital controller device.
    kp = 0
    ki = 0
    kd = 0
    sample_rate = 1.0/100
    max_signal = 12 # da capire
    target = 0 # target del pid, valore agnostico
    

    def __init__(self, kp, ki, kd, max_signal, sample_rate, target):
        
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if target is not None:
            self.target = target # distanza in cm
        if sample_rate is not None:
            self.sample_rate = sample_rate
        if max_signal is not None:
            self.max_signal = max_signal # initial assumption

        self.signal = 0
        self.accumulator = 0
        self.last_reading = 0
        self.xa = 0.140 # in m 



    # Set_new_target sets the target attribute and resets the held accumulator value. Resetting the accumulator
    # is necessary for the integral controller to function properly.
    def set_new_target(self, target):
        self.accumulator = 0
        self.target = target

        # return target, accumulator;
    

    # Adjust_signal is the method that performs the actual function of the controller. This method calculates a 
    # new signal based on the feedback_value, accumulator value, last_reading, and target. It then sets the new
    # signal to the self.signal attribute.
    def adjust_signal(self, feedback_value):
        # kp = 100 #initial assumption
        # ki = 0
        # kd = 0
        # Calculate the error - difference between target and feedback_value (measurement)
        error = self.target - feedback_value

        # Add error to accumulator
        self.accumulator += error * self.sample_rate

        # Calculate signal based on PID coefficients
        self.signal = self.kp * error + self.ki * self.accumulator + self.kd * (error-self.last_reading)/self.sample_rate

        # max_signal = 10000000000
        # If calculated signal exceeds max_signal, then set signal to max_signal value. Do this in both positive
        # and negative directions
        if self.signal > self.max_signal:
            self.signal = self.max_signal
        elif self.signal < -self.max_signal:
            self.signal = -self.max_signal

        # Save current feedback_value as last_reading attribute for use in calulating next signal.
        self.last_reading = error

        return self.signal

        # Run_simulation is a controller method which will simulate the output of a system object "system_simulator"
        # over a time_period "duration."

    def get_force_z(self, signal):
        force_z = signal # Newton
        return force_z

    def get_force_lateral(self, signal_distance, signal_psi):
        force_L = (signal_distance*self.xa - signal_psi)/(2 * self.xa) # Newton
        force_R = (signal_distance*self.xa + signal_psi)/(2 * self.xa) # Newton
        return force_L, force_R

    # Le funzioni PWM dovrebbero darmi dei valori da 0 a 100 di motore 
    # Se il signal � 12, la forza � 12 N e voglio una PWM di 100
    def pwm_z_motor(self, force_z):
        max_z_force = 0.1
        m_coefficient = 100/max_z_force #  da specificare per ogni motore
        pwm_z = (force_z * m_coefficient) 

        pwm_z_limit = 40

        if pwm_z > pwm_z_limit:
            pwm_z = pwm_z_limit
        elif pwm_z < -pwm_z_limit:
            pwm_z = -pwm_z_limit
        return pwm_z

    def pwm_L_motor(self, force_L):
        max_L_force = 0.1
        m_coefficient = 100/max_L_force # da specificare per ogni motore
        pwm_L = (force_L * m_coefficient) 

        pwm_L_limit = 40

        if pwm_L > pwm_L_limit:
            pwm_L = pwm_L_limit
        elif pwm_L < -pwm_L_limit:
            pwm_L = -pwm_L_limit
        return pwm_L

    def pwm_R_motor(self, force_R):
        max_R_force = 0.1
        m_coefficient = 100/max_R_force # da specificare per ogni motore
        pwm_R = (force_R * m_coefficient) 

        pwm_R_limit = 40

        if pwm_R > pwm_R_limit:
            pwm_R = pwm_R_limit
        elif pwm_R < -pwm_R_limit:
            pwm_R = -pwm_R_limit
        return pwm_R


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


def psi_mean(psi_list,psi_meas):
    sum_psi = 0
    for i in range(len(psi_list)):
        sum_psi = sum_psi + (psi_list[i] - psi_meas)**2
    
    if sum_psi/len(psi_list) >= 50: # se la varianza è maggiore di 50
        psi = psi_meas
    else:
        psi = sum(psi_list)/len(psi_list)
    
    return psi


def pos_mean(pos_list,pos_meas):
    mean_pos = np.mean(pos_list)
    flag = 0
    #sum_pos = 0

    #for i in range(len(pos_list)):
    #    sum_pos = sum_pos + (pos_list[i] - mean_pos)**2
    
    #var_pos = sum_pos/len(pos_list)
    #print(var_pos)
    #print(pos_meas, mean_pos + 400*var_pos, mean_pos - 400*var_pos)
    #if (pos_meas <= (mean_pos + 400*var_pos)) and (pos_meas >= (mean_pos - 400*var_pos)):
    #    pos_list.append(pos_meas)
    #    pos_list.pop(0)

    if (pos_meas <= (mean_pos + 0.2)) and (pos_meas >= (mean_pos - 0.2)):
        pos_list.append(pos_meas)
        pos_list.pop(0)
    else:
        flag = 1
    
    return pos_list, flag

def z_mean(z_list,z_meas):
    mean_z = np.mean(z_list)
    #sum_pos = 0

    #for i in range(len(pos_list)):
    #    sum_pos = sum_pos + (pos_list[i] - mean_pos)**2
    
    #var_pos = sum_pos/len(pos_list)
    #print(var_pos)
    #print(pos_meas, mean_pos + 400*var_pos, mean_pos - 400*var_pos)
    #if (pos_meas <= (mean_pos + 400*var_pos)) and (pos_meas >= (mean_pos - 400*var_pos)):
    #    pos_list.append(pos_meas)
    #    pos_list.pop(0)

    if (z_meas <= (mean_z + 0.2)) and (z_meas >= (mean_z - 0.2)):
        z_list.append(z_meas)
        z_list.pop(0)
    else:
        flag = 1
    
    return z_list, flag



def imu_to_uwb(accelerometer, gyro, mag):
    acc_uwb = [accelerometer[1], accelerometer[0], - accelerometer[2]]
    gyro_uwb = [gyro[1], gyro[0], - gyro[2]]
    mag_uwb = [mag[1], mag[0], - mag[2]]
    return acc_uwb, gyro_uwb, mag_uwb

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, math.cos(theta),-math.sin(theta)],
                   [ 0, math.sin(theta), math.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ math.cos(theta), 0, math.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-math.sin(theta), 0, math.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ math.cos(theta), -math.sin(theta), 0 ],
                   [ math.sin(theta), math.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def rotation_UWB(acc, gyro, angles):
    
    R = np.transpose(Rx(angles[0]*np.pi/180)@Ry(angles[1]*np.pi/180)@Rz(angles[2]*np.pi/180))

    #acc = np.transpose(acc)
    #gyro = np.transpose(gyro)

    acc_UWB = R@acc

    gyro_UWB = R@gyro
    
    return acc_UWB, gyro_UWB
