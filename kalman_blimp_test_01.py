# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y, z directions,
# the acceleration in the same directions and the yaw orientation and angular velocity.
# Jan 23
# Riccardo Tessarin, Federico Marchi

import numpy as np

# Definition of the matrices used in 
# Kalman filter
class kalman_blimp: 
    dt = 1/100
    xR = 0.05 #m distance of R motor from CG
    xL = 0.05 #m distance of R motor from CG
    m = 0.25 #kg total mass of airship
    Fl = 0 # N forces of the motors
    Fr = 0
    Fu = 0

    def __init__(self, dt=None, Fl= None, Fr=None, Fu= None):
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
    
    F = np.array([[1, (dt^2)/2, 0, 0, 0, 0, 0, 0 ],
                [0, 1, 0, 0, 0, 0, 0, 0], 
                [0, 0, 1, (dt^2)/2, 0, 0, 0, 0 ],
                [0, 0, 0, 1, 0, 0, 0, 0 ],
                [0, 0, 0, 0, 1, (dt^2)/2, 0, 0 ],
                [0, 0, 0, 0, 0, 1, 0, 0 ], 
                [0, 0, 0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 0, 0, 1]])

    B = np.array([[0, 0, 0],
                  [1, 1, 0],
                  [0, 0, 0],
                  [0, 0, 0],
                  [0, 0, 0],
                  [0, 0, 1],
                  [0, 0, 0],
                  [xR, xL, 0]])

    u_t = np.array([Fl/m, Fr/m, Fu/m]) #input vector depending on force