# Prove di codice per fare la trilaterazione
# 13 Jan 23
# Author Riccardo Tessarin
import math
from math import dist
import numpy as np
import matplotlib.pyplot  as plt


# Anchor definition, set their position in space

A_n1 = np.array([0.00, 7.19, 2.15])
A_n2 = np.array([0.00, 3.62, 3.15])
A_n3 = np.array([0.00, 0.00, 2.15])
A_n4 = np.array([4.79, 1.85, 3.15])
A_n5 = np.array([4.79, 5.45, 2.15])
A_n6  = np.array([3.00, 9.35, 3.15])

An =  A_n1, A_n2, A_n3, A_n4, A_n5, A_n6
An = np.array(An)

#print(An[0,2]) #reminder per come chiamare gli elementi nel numpy array

# The target point to track
# Initially set still
T = np.array([1., 1., 1.])

# Distance from each Anchor, in the final code this parameters was computed by the UWB network
d1 = dist(A_n1,T) + np.random.normal(loc=0.0, scale=0.3)
#print('d1  = ', d1)
d2 = dist(A_n2,T) + np.random.normal(loc=0.0, scale=0.05)
d3 = dist(A_n3,T) + np.random.normal(loc=0.0, scale=0.05)
d4 = dist(A_n4,T) + np.random.normal(loc=0.0, scale=0.3)
d5 = dist(A_n5,T) + np.random.normal(loc=0.0, scale=0.05)
d6 = dist(A_n6,T) + np.random.normal(loc=0.0, scale=0.05)
#d7 = dist(A_n7,T) + np.random.normal(loc=0.0, scale=0.1)

d = np.array([ d1, d2, d3, d4, d5, d6])
#d = np.array(d)
n = ["A_1", "A_2", "A_3", "A_4", "A_5", "A_6"]
print ("d=", d)
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
    
#for c in range(len(x)-3, len(x)):
 #   A[c-1, :] = [0, 0, 0, x[c]-x[0], y[c]-y[0], z[c]-z[0]]

#for c in range(1,len(x)):
 #   b[c-1] = d[0]**2 - d[c]**2 + x[c]**2 + y[c]**2 + z[c]**2 - x[0]**2 -y[0]**2 -z[0]**2


print("A = ", A)
print("b= ", b)

# Calculation of the position with the LLS Alghorithm
pos = np.matmul(np.linalg.pinv(A), b)/2
pos_str = str((np.round(pos[0,0],2), np.round(pos[1,0]),np.round(pos[2,0])))

print("posizione 0 = ", pos[1])

# LS error calculation
x = np.transpose(pos) - T
print ('T = ', T)
e_LS = np.linalg.norm(x, ord=2)

print('pos_LS = ', np.transpose(pos))
print('e_LS = ', e_LS )



# WLS implementation
# The W is diagonal and filled with the values of the variance of each measure
w1 = 1/0.3**2
w2 = 1/0.05**2
w3=w2
w4= 1/0.3**2
w5=w6=w2
W = np.diag([w1, w2, w3, w4, w5])


pos_wls = np.matmul(np.matmul(np.matmul(np.matmul(np.linalg.pinv(np.matmul(np.matmul(np.matmul(A.T, W.T),W),A)),A.T),W.T),W),b)/2
# Alternative
#pos_wls = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(A.T,W),A),A.T),W),b)/2

print('pos WLS = ', np.transpose(pos_wls))
pos_str_wls = str((np.round(pos_wls[0,0],2), np.round(pos_wls[1,0]),np.round(pos_wls[2,0])))

y = np.transpose(pos_wls) - T
print("y=", y[0])
e_WLS = np.linalg.norm(y, ord=2)

print("e_WLS = ", e_WLS)

# Print the figure
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(An[:,0], An[:,1], An[:,2], marker= '^')
ax.scatter(T[0], T[1], T[2], marker= 'o')
ax.scatter(pos[0,0], pos[1,0], pos[2,0], marker='o')
ax.scatter(pos_wls[0,0], pos_wls[1,0], pos_wls[2,0], marker='o')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
for i, txt in enumerate(n):
    ax.text(An[i,0], An[i,1], An[i,2], txt)
ax.text(pos[0,0], pos[1,0], pos[2,0], pos_str )
ax.text(pos_wls[0,0], pos_wls[1,0], pos_wls[2,0], pos_str_wls )
ax.text(T[0], T[1], T[2], "T" )
plt.show()



