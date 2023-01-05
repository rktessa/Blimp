# Kalman filter for Blimp tracking motion
# The implemented Kalman filter traks the motion in x, y, z directions,
# the acceleration in the same directions and the yaw orientation and angular velocity.
# Jan 23
# Riccardo Tessarin, Federico Marchi

import numpy as np

# Definition of the matrices used in 
# Kalman filter
class kalman_blimp: 
    
    F = np.array([[1, (dt^2)/2, 0, 0, 0, 0, 0, 0 ],
                [0, 1, 0, 0, 0, 0, 0, 0], 
                [0, 0, 1, (dt^2)/2, 0, 0, 0, 0 ],
                [0, 0, 0, 1, 0, 0, 0, 0 ],
                [0, 0, 0, 0, 1, (dt^2)/2, 0, 0 ],
                [0, 0, 0, 0, 0, 1, 0, 0 ], 
                [0, 0, 0, 0, 0, 0, 1, dt],
                [0, 0, 0, 0, 0, 0, 0, 1]])