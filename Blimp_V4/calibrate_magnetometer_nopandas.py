import numpy as np
import csv
from scipy import linalg
import time
import board
from adafruit_icm20x import ICM20948, MagDataRate

"""Launching this script we can collect the data for calibratrion, perform calibration
and save the result in a csv file, that can be then used by calibration function
to correct the values of magnetometer during live misurations."""


class Magnetometer(object):
    
    '''
        To obtain Gravitation Field (raw format):
    1) get the Total Field for your location from here:
       http://www.ngdc.noaa.gov/geomag-web (tab Magnetic Field)
       es. Total Field = 47,241.3 nT | my val :47'789.7   Povo = 48,026.0 nT in gauss = 0.48026 G 
    2) Convert this values to Gauss (1nT = 10E-5G)
       es. Total Field = 47,241.3 nT = 0.47241G   ==> 1/ 0,001 5 inverso = 666.67 
    3) Convert Total Field to Raw value Total Field, which is the
       Raw Gravitation Field we are searching for
       Read your magnetometer datasheet and find your gain value,
       Which should be the same of the collected raw points
       es. on HMC5883L, given +_ 1.3 Ga as Sensor Field Range settings
           Gain (LSB/Gauss) = 1090 
           Raw Total Field = Gain * Total Field 
           Povo = 0.48026 * 666.67 = 320.17
           

     references :
        -  https://teslabs.com/articles/magnetometer-calibration/      
        -  https://www.best-microcontroller-projects.com/hmc5883l.html

    '''
    MField = 320
    file = "data_for_calibration.txt"

    def __init__(self, F=MField, file= None ): 


        # initialize values
        
        self.F   = F
        self.b   = np.zeros([3, 1]) #vector for hard iron calibration
        self.A_1 = np.eye(3) # matrix for soft iron calibration
        if file is not None:
            self.file = file

            
    def run(self):
        
        data = np.loadtxt(self.file,delimiter=',') #input
        print("shape of data:",data.shape)
        
        
        # ellipsoid fit
        s = np.array(data).T
        M, n, d = self.__ellipsoid_fit(s)

        # calibration parameters
        M_1 = linalg.inv(M)
        self.b = -np.dot(M_1, n)
        self.A_1 = np.real(self.F / np.sqrt(np.dot(n.T, np.dot(M_1, n)) - d) * linalg.sqrtm(M))
        
        # Salvare i risultati in un file "data.csv" usando pandas 
        with open('data.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "z"])
            writer.writerow([self.b[0,0],self.b[1,0], self.b[2,0]])
            writer.writerow([self.A_1[0,0], self.A_1[1,0], self.A_1[2,0]])
            writer.writerow([self.A_1[0,1], self.A_1[1,1], self.A_1[2,1]])
            writer.writerow([self.A_1[0,2], self.A_1[1,2], self.A_1[2,2]])


        #df_cal = pd.DataFrame(np.array([[self.b[0,0], self.b[1,0], self.b[2,0]], [self.A_1[0,0], self.A_1[1,0], self.A_1[2,0]], [self.A_1[0,1], self.A_1[1,1], self.A_1[2,1]], [self.A_1[0,2], self.A_1[1,2], self.A_1[2,2]]]),
                                #columns= ['x', 'y', 'z'])
        #df_cal.to_csv("data.csv")
        
        
        print("Soft iron transformation matrix:\n",self.A_1)
        print("Hard iron bias:\n", self.b)
        
        



    def __ellipsoid_fit(self, s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -3,  0,  0],
                      [ 0,  0,  0,  0, -3,  0],
                      [ 0,  0,  0,  0,  0, -3]])

        # v_1 (eq. 15, solution)
        E = np.dot(np.linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(np.linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d
        
        
        
if __name__=='__main__':

    """
    The function ask to move in all the direction for 60
    seconds the blimp in order to aquire data from the 
    magnetometer
    """

    i2c = board.I2C()  # uses board.SCL and board.SDA
    icm = ICM20948(i2c)
    icm.magnetometer_data_rate = MagDataRate.RATE_100HZ     
    mag_file = open("data_for_calibration.txt","w+")
    data_string = ""
    print("Start move the blimp gondola in all directions, describing an 8")
    time.sleep(3)

    tempo_inizio = time.perf_counter()
    while  time.perf_counter() < (tempo_inizio +60):
        data_string = str(icm.magnetic[0]) + " ," + str(icm.magnetic[1]) + " ," + str(icm.magnetic[2]) + "\n"
        mag_file.write(data_string)
        tempo_attuale = time.perf_counter()
        print("Time= ", round(tempo_attuale-tempo_inizio,2))


    print("Ending Acquisition")
    time.sleep(1)

    Magnetometer().run()
    print("Saved Calibrated values for Hard and Soft Iron in data.csv")
    time.sleep(3)
