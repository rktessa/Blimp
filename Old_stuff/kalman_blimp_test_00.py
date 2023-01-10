import math
import numpy as np
from numpy.random import randn
from filterpy.kalman import predict
from filterpy.common import Q_discrete_white_noise

# Tracking a Blimp in 3D space, this first prove use filterpy. 

def compute_blimp_data(z_var, process_var, count=1, dt=1.):
    "returns track, measurements 1D ndarrays"
    x, vel = 0., 1.
    z_std = math.sqrt(z_var) #sensor position variance
    p_std = math.sqrt(process_var) #Velocity variance
    xs, zs = [], []
    for _ in range(count):
        v = vel + (randn() * p_std * dt)
        x += v*dt        
        xs.append(x) #state array
        zs.append(x + randn() * z_std) #measurement array       
    return np.array(xs), np.array(zs)

dt = 0.1
x = np.array([10.0, 4.5])
P = np.diag([500, 49])
F = np.array([[1, dt], [0, 1]])
B = 0.  # my dog doesn't listen to me!
u = 0
Q = Q_discrete_white_noise(dim=2, dt=1., var=2.35)
# Q is the process noise

x, P = predict(x, P, F, Q, B, u)
print('x =', x)
print('P =', P)






