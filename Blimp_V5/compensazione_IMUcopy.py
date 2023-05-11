# Try to compensate the IMU to translate its lectures on the global reference frame
# 9 May 2023
# Riccardo Tessarin

import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin 
from blimp_class import Madgwick
from quaternion import Quaternion

# Provo a stampare gli output per vedere come i sono i dati

f = open("C:\Volume_D\Programming\Blimp_git\Blimp\Blimp_V5\log_IMU_static_blimp.txt", "r")
Lines = f.readlines()

accelerations = []
acc_global = []
gyro = []
mag = []
times = []
angle = []




# Initialization of Madgwick filter
q0 = Quaternion( -0.05688804,  0.2823188,  -0.9575169,  -0.01487385)
mad = Madgwick(sampleperiod = 1/12, quaternion=q0, beta=0.081)

# Funzione matrice di rotazione, va in rad
def J(roll, pitch, yaw):
    J = np.array([[cos(pitch)*cos(roll), -sin(roll)*cos(pitch)+sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw)],
                  [cos(pitch)*sin(yaw), cos(yaw) *cos(roll) + sin(roll)* sin(pitch)* sin(yaw), -sin(roll)* cos(yaw) + cos(roll) *sin(pitch)* sin(yaw) ],
                  [-sin(pitch), cos(pitch)*sin(roll), cos(roll)*cos(pitch)]])
    return J

def W(wx, wy, wz):
    W = np.matrix([[0, -wz, wy],
                   [wz, 0, -wx],
                   [-wy, wx, 0]])
    return W

for line in Lines: 
    time, accX, accY, accZ, gyrX, gyrY,gyrZ, magX, magY, magZ  = line.split(" ")
    # ! nota bene
    # Sto allineando le letture della imu al global reference frame 
    accelerations.append((float(accX),float(accY),float(accZ)))
    gyro.append((float(gyrX),float(gyrY),float(gyrZ)))
    mag.append((float(magX),float(magY),float(magZ)))
    times.append(float(time))

accelerations = np.matrix(accelerations)
gyro = np.matrix(gyro) 
mag = np.matrix(mag)
times = np.array(times)

# per inseire ulteriore termine inizializzo vettore velocità 
velocity = np.zeros((len(accelerations), 3))

# CALIBRAZIONE STATICA, SI FA SOLO CON ACQUISIZIONE STATICA
gyro_error = np.mean(gyro, axis=0) #[[-0.01059974  0.00730897  0.00019453]]
gyro_variance = np.var(gyro, axis=0) # [9.17014654e-06 5.35342037e-05 8.37444167e-06]
print("gyro", gyro_error, gyro_variance)
acc_error = np.mean(accelerations, axis=0) #
acc_variance = np.var(accelerations, axis=0) #
print("accel", acc_error, acc_variance)

# REWRITE THE GYRO SUBTRACTING THE OFFSETS
# Il segno della y è cambiato perchè è l'unico asse che non ho ruotato
gyro = gyro + np.array([ 0.00295614, -0.00333192, -0.00058075])

# Uso Madgwick e calcolo la pose
for i in range(len(accelerations)-1):
    delta_t = times[i+1] - times[i]
    mad.samplePeriod = delta_t 
    #print(1/mad.samplePeriod)
    quaternion = mad.update(gyro[i,:], accelerations[i,:], mag[i,:])
    quat = Quaternion(quaternion)            
    roll, pitch, yaw = quat.to_euler123()  # Results are in rad
    angle.append((roll, pitch, yaw))

    
    


    # Uso gli angoli per cercare di ricostruire una rotazione ruotata
    # del vettore accelerazione sui tre assi x, y, z
    gyro_term = -2 * W(gyro[i,0], gyro[i,1], gyro[i,2]) @ J(roll, pitch, yaw) @ np.transpose(velocity[i,:]) #termine dovuto al gyro
    acc_abs = np.transpose(J(roll, pitch, yaw))@ (np.transpose(accelerations[i,:])+np.transpose(gyro_term)) - np.array([[0.53565487, 0.18989142, 9.68873689]]).T
    acc_global.append((acc_abs[0,0], acc_abs[1,0], acc_abs[2,0]))
    
    
    # Integro le accelerazioni per calcolare le velocità
    velocity[i+1,:] = velocity[i,:] + np.transpose(acc_abs)*delta_t 


print(quat)
print(cos(0))
#print(acc_global + np.array([[0, 0, 9.81]]))

somma_accel = acc_global + np.array([[0, 0, 9.81]])
gfinale = np.mean(somma_accel, axis= 0)
print ("accel_ruotate medie = ", gfinale)

print(np.transpose(J(roll, pitch, yaw)).shape)
print(np.transpose(accelerations[i,:]).shape)
print(np.array([[0, 0, 9.80596]]).T.shape)
print(acc_abs[0,0])

# I plot alla fine
#converto in np array
acc_global = np.array(acc_global)

#acc_global = np.transpose(acc_global)
#print(acc_global)
fig, (ax1, ax2,ax3) = plt.subplots(3)
fig.subplots_adjust(hspace=0.5)
ax1.plot(times, accelerations[:, 0])
ax1.plot(times[0:len(times)-1], acc_global[:, 0])
ax2.plot(times, accelerations[:, 1])
ax2.plot(times[0:len(times)-1], acc_global[:, 1])
ax3.plot(times, accelerations[:, 2])
ax3.plot(times[0:len(times)-1], acc_global[:, 2])
ax1.set_title('X acceleration')
ax2.set_title('Y acceleration')
ax3.set_title('Z acceleration')
ax1.grid()
ax2.grid()
ax3.grid()

# I plot alla fine
# converto in array angle
angle = np.array(angle)
fig2, (ax1, ax2,ax3) = plt.subplots(3)
fig2.subplots_adjust(hspace=0.5)
ax1.plot(times[0:len(times)-1], angle[:, 0]*180/np.pi)
ax2.plot(times[0:len(times)-1], angle[:, 1]*180/np.pi)
ax3.plot(times[0:len(times)-1], angle[:, 2]*180/np.pi)
ax1.set_title('Roll')
ax2.set_title('Pitch')
ax3.set_title('Yaw')
ax1.grid()
ax2.grid()
ax3.grid()


fig3, (ax1, ax2,ax3) = plt.subplots(3)
fig3.subplots_adjust(hspace=0.5)
ax1.plot(times, gyro[:, 0])
ax2.plot(times, gyro[:, 1])
ax3.plot(times, gyro[:, 2])
ax1.set_title('X gyro')
ax2.set_title('Y gyro')
ax3.set_title('Z gyro')
ax1.grid()
ax2.grid()
ax3.grid()

fig4, (ax1, ax2,ax3) = plt.subplots(3)
fig4.subplots_adjust(hspace=0.5)
ax1.plot(times, velocity[:, 0])
ax2.plot(times, velocity[:, 1])
ax3.plot(times, velocity[:, 2])
ax1.set_title('X velocity')
ax2.set_title('Y velocity')
ax3.set_title('Z velocity')
ax1.grid()
ax2.grid()
ax3.grid()



plt.show()
