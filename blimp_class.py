
import warnings
import time
import numpy as np
from numpy.linalg import norm
from quaternion import Quaternion
import pandas as pd
from matplotlib import pyplot as plt




#   FUNCTION AND CLASS USED FOR ESTIMATE THE BLIMP ORIENTATION IN SPACE

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
    and soft iron offsets. 
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


def orientation_initial(raw_acc, raw_gyr, raw_mag):

    q0 = Quaternion(0, 0, 0, 1)
    mad = Madgwick(sampleperiod = 1/100, quaternion=q0, beta=1)
    roll_vec = []
    pitch_vec = []
    yaw_vec = []

    time_zero = time.perf_counter()
    time_start = time.perf_counter()
    while  time.perf_counter() < (time_start +10):
        #Aquisizione magnetometro e calibrazione dei dati:
        mag = calibration(raw_mag)
        #Creazione vettore input per classe madgwick
        accelerometer, gyroscope, magnetometer = np.asarray(raw_acc), np.asarray(raw_gyr), np.asarray(mag)
        time_fine = time.perf_counter()
        #setting the variable frequency of updateof Madgwick alghorithm
        mad.samplePeriod = time_fine - time_zero
        quaternion = mad.update(gyroscope, accelerometer, magnetometer)
        time_zero = time.perf_counter()
        quat = Quaternion(quaternion)
        #print("res = ", str(quaternion))
        roll, pitch, yaw = quat.to_euler123()  # Result is in rad
        print(roll, pitch, yaw)
        roll_vec.append(roll)
        pitch_vec.append(pitch)
        yaw_vec.append(yaw)
    
    roll_0 = sum(roll_vec[-40 :])/40 #perform the mean of the last 20 element
    pitch_0 = sum(pitch_vec[-40 :])/40
    yaw_0 = sum(yaw_vec[-40 :])/40
    
    return roll_0, pitch_0, yaw_0 # These are the values of initial angles


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
    target = 0 # target de pid, valore agnostico
    

    def __init__(self, kp, ki, kd, max_signal, sample_rate, target):
        
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
        if target is not None:
            self.target = target #distanza in cm
        if sample_rate is not None:
            self.sample_rate = sample_rate
        if max_signal is not None:
            self.max_signal = max_signal #initial assumption

        self.signal = 0
        self.accumulator = 0
        self.last_reading = 0
        self.xa = 13.5 #in cm 



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
        self.signal = self.kp * error + self.ki * self.accumulator + self.kd * (feedback_value - self.last_reading)/self.sample_rate

        # max_signal = 10000000000
        # If calculated signal exceeds max_signal, then set signal to max_signal value. Do this in both positive
        # and negative directions
        if self.signal > self.max_signal:
            self.signal = self.max_signal
        elif self.signal < -self.max_signal:
            self.signal = -self.max_signal

        # Save current feedback_value as last_reading attribute for use in calulating next signal.
        self.last_reading = feedback_value

        return self.signal

        # Run_simulation is a controller method which will simulate the output of a system object "system_simulator"
    # over a time_period "duration."

    def get_force_z(self, signal):
        force_z = signal
        return force_z

    def get_force_lateral(self, signal_distance, signal_psi):
        force_L = (signal_distance*self.xa + signal_psi)/(2 * self.xa)
        force_R = (signal_distance*self.xa - signal_psi)/(2 * self.xa)
        return force_L, force_R

    def pwm_z_motor(self, force_z):
        m_coefficient = 11.99/100 #  da specificare per ogni motore
        pwm_z = (force_z / m_coefficient) 
        return pwm_z

    def pwm_L_motor(self, force_L):
        m_coefficient = 11.99/100 # da specificare per ogni motore
        pwm_L = (force_L * m_coefficient) 
        return pwm_L

    def pwm_R_motor(self, force_R):
        m_coefficient = 11.99/100 # da specificare per ogni motore
        pwm_R = (force_R * m_coefficient) 
        return pwm_R







