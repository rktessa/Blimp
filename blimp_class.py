
import warnings
import numpy as np
from numpy.linalg import norm
from quaternion import Quaternion
import pandas as pd
from matplotlib import pyplot as plt

class Madgwick:
    samplePeriod = 1/100
    quaternion = Quaternion(0, 0, 1, 0)
    beta = 1
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
    and sof iron offsets. 
    """
    magnetic = np.array(magnetic, dtype=float).flatten() #Convert the measure in a numpy array
    
    df = pd.read_csv('data.csv')
    
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