# Prove di codice per fare la trilaterazione
# 13 Jan 23
# Author Riccardo Tessarin
import math
from math import dist
import numpy as np
import matplotlib.pyplot  as plt


# Anchor definition, set their position in space

An_1 = np.array([0.1, 0.2, 1.55]) 
An_2 = np.array([0.3, 3.15, 1.65]) 
An_3 = np.array([0.15, 6.2, 1.60] )
An_4 = np.array([3.2, 0.1, 1.59] )
An_5 = np.array([3.1, 3.25, 1.57] )
An_6 = np.array([3.0, 6.15, 1.56])
An_7 = np.array([2.2, 2.25, 2.32])

An = [ An_1, An_2, An_3, An_4, An_5, An_6, An_7]
An = np.array(An)

#print(An[0,2]) #reminder per come chiamare gli elementi nel numpy array

# The target point to track
# Initially set still
T = np.array([2., 5., 1.])

# Distance from each Anchor, in the final code this parameters was computed by the UWB network
d1 = dist(An_1,T) + np.random.normal(loc=0.0, scale=0.1)
#print('d1  = ', d1)
d2 = dist(An_2,T) + np.random.normal(loc=0.0, scale=0.1)
d3 = dist(An_3,T) + np.random.normal(loc=0.0, scale=0.1)
d4 = dist(An_4,T) + np.random.normal(loc=0.0, scale=0.1)
d5 = dist(An_5,T) + np.random.normal(loc=0.0, scale=0.1)
d6 = dist(An_6,T) + np.random.normal(loc=0.0, scale=0.1)
d7 = dist(An_7,T) + np.random.normal(loc=0.0, scale=0.1)

d = [ d1, d2, d3, d4, d5, d6, d7]
d = np.array(d)
n = ["A_1", "A_2", "A_3", "A_4", "A_5", "A_6", "A_7"]

# Compute the position of T using trilateration and LS formula
# Defining initial A matrix and b vector
A = np.zeros((6,3))
b = np.zeros((6,1))

# Definition of vectors X, Y, Z, first position is the target,
# the others the anchors coordinates
x = An[:,0]
y = An[:,1]
z = An[:,2]

# Calculation of A and b for the case with 7 anchors
for c in range(1, len(x)):
    A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0]] 
    #A[c-1, :] = [x[c]-x[0], y[c]-y[0], z[c]-z[0], 0, 0, 0]  
    b[c-1] = d[0]**2 - d[c]**2 + x[c]**2 + y[c]**2 + z[c]**2 - x[0]**2 -y[0]**2 -z[0]**2   
    
#for c in range(len(x)-3, len(x)):
 #   A[c-1, :] = [0, 0, 0, x[c]-x[0], y[c]-y[0], z[c]-z[0]]

#for c in range(1,len(x)):
 #   b[c-1] = d[0]**2 - d[c]**2 + x[c]**2 + y[c]**2 + z[c]**2 - x[0]**2 -y[0]**2 -z[0]**2


print(A)
print(b)

# Calculation of the position with the LLS Alghorithm
pos = np.matmul(np.linalg.pinv(A), b)/2
pos_str = str((np.round(pos[0,0],2), np.round(pos[1,0]),np.round(pos[2,0])))
print('pos = ', np.transpose(pos))

# LS error calculation
x = np.transpose(pos) - T
print ('T = ', T)
e_LS = np.linalg.norm(x, ord=2)
print('x =', x)
print('e_LS = ', e_LS )



# WLS implementation
# Noise of the distance measured with variance 0.1 m 
# random white noise added to the measured distaces
w1 = np.random.normal(loc=0.0, scale=1.0)
w2 = np.random.normal(loc=0.0, scale=1.0)
w3 = np.random.normal(loc=0.0, scale=1.0)
w4 = np.random.normal(loc=0.0, scale=1.0)
w5 = np.random.normal(loc=0.0, scale=1.0)
w6 = np.random.normal(loc=0.0, scale=1.0)
w7 = np.random.normal(loc=0.0, scale=1.0)

W = np.diag([w1, w2, w3, w4, w5, w6, w7])




# Print the figure
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(An[:,0], An[:,1], An[:,2], marker= '^')
ax.scatter(T[0], T[1], T[2], marker= 'o')
ax.scatter(pos[0,0], pos[1,0], pos[2,0], marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
for i, txt in enumerate(n):
    ax.text(An[i,0], An[i,1], An[i,2], txt)
ax.text(pos[0,0], pos[1,0], pos[2,0], pos_str )
ax.text(T[0], T[1], T[2], "T" )
plt.show()



