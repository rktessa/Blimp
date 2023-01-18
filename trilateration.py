# Prove di codice per fare la trilaterazione
# 13 Jan 23
# Author Riccardo Tessarin
import math
from math import dist
import numpy as np
import matplotlib.pyplot  as plt


# Anchor definition, set their position in space

An_1 = [0.1, 0.2, 1.55] 
An_2 = [0.3, 3.15, 1.65] 
An_3 = [0.15, 6.2, 1.60] 
An_4 = [3.2, 0.1, 1.59] 
An_5 = [3.1, 3.25, 1.57] 
An_6 = [3.0, 6.15, 1.56]
An_7 = [2.2, 2.25, 2.32]

An = np.array([ An_1, An_2, An_3, An_4, An_5, An_6, An_7])

#print(An[0,2]) #reminder per come chiamare gli elementi nel numpy array

# The target point to track
# Initially set still
T = [2., 5., 1.]

# Distance from each Anchor, in the final code this parameters was computed by the UWB network
d1 = dist(An_1,T)
#print('d1  = ', d1)
d2 = dist(An_2,T)
d3 = dist(An_3,T)
d4 = dist(An_4,T)
d5 = dist(An_5,T)
d6 = dist(An_6,T)
d7 = dist(An_7,T)

d = np.array([ d1, d2, d3, d4, d5, d6, d7])
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
#pos = np.matmul(np.linalg.inv(A),b)/2
print('pos = ', pos)

# Print the figure
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(An[:,0], An[:,1], An[:,2], marker= '^')
ax.scatter(T[0], T[1], T[2], marker= 'o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
for i, txt in enumerate(n):
    ax.text(An[i,0], An[i,1], An[i,2], txt)
plt.show()

