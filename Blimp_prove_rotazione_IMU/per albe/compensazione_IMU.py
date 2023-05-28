# Try to compensate the IMU to translate its lectures on the global reference frame
# 9 May 2023
# Riccardo Tessarin

import matplotlib.pyplot as plt
import numpy as np
from numpy import cos, sin 
from blimp_class import Madgwick
from quaternion import Quaternion

# Provo a stampare gli output per vedere come i sono i dati

f = open("C:\Volume_D\Programming\Blimp_git\Blimp\Blimp_V5\log_IMU_static.txt", "r")
Lines = f.readlines()

accelerations = []
acc_global = []
gyro = []
mag = []
times = []
angle = []

# Initialization of Madgwick filter
q0 = Quaternion( 4.27366350e-04, -9.17140257e-01, -3.98237687e-01,  1.61341449e-02)
mad = Madgwick(sampleperiod = 1/12, quaternion=q0, beta=0.005)

# Funzione matrice di rotazione, va in rad
def J(roll, pitch, yaw):
    J = np.array([[cos(pitch)*cos(roll), -sin(roll)*cos(pitch)+sin(roll)*sin(pitch)*cos(yaw), sin(roll)*sin(yaw) + cos(roll)*sin(pitch)*cos(yaw)],
                  [cos(pitch)*sin(yaw), cos(yaw) *cos(roll) + sin(roll)* sin(pitch)* sin(yaw), -sin(roll)* cos(yaw) + cos(roll) *sin(pitch)* sin(yaw) ],
                  [-sin(pitch), cos(pitch)*sin(roll), cos(roll)*cos(pitch)]])
    return J

for line in Lines: 
    time, accX, accY, accZ, gyrX, gyrY,gyrZ, magX, magY, magZ  = line.split(" ")
    # ! nota bene
    # Sto allineando le letture della imu al global reference frame 
    accelerations.append((-1*float(accX),float(accY),-1*float(accZ)))
    gyro.append((-1*float(gyrX),float(gyrY),-1*float(gyrZ)))
    mag.append((-1*float(magX),float(magY),-1*float(magZ)))
    times.append(float(time))

accelerations = np.matrix(accelerations)
gyro = np.matrix(gyro) 
mag = np.matrix(mag)
times = np.array(times)

# CALIBRAZIONE STATICA, SI FA SOLO CON ACQUISIZIONE STATICA
gyro_error = np.mean(gyro, axis=0) #[[-0.01059974  0.00730897  0.00019453]]
gyro_variance = np.var(gyro, axis=0) # [9.17014654e-06 5.35342037e-05 8.37444167e-06]
print("gyro", gyro_error, gyro_variance)
acc_error = np.mean(accelerations, axis=0) #
acc_variance = np.var(accelerations, axis=0) #
#print("accel", acc_error, acc_variance)

# REWRITE THE GYRO SUBTRACTING THE OFFSETS
# Il segno della y è cambiato perchè è l'unico asse che non ho ruotato
gyro = gyro + np.array([-0.01059974,  -0.00730897,  0.00019453])

# Uso Madgwick e calcolo la pose
for i in range(len(accelerations)-1):
    mad.samplePeriod = times[i+1] - times[i]
    #print(1/mad.samplePeriod)
    quaternion = mad.update(gyro[i,:], accelerations[i,:], mag[i,:])
    quat = Quaternion(quaternion)            
    roll, pitch, yaw = quat.to_euler123()  # Results are in rad
    angle.append((roll, pitch, yaw))
    # Uso gli angoli per cercare di ricostruire una rotazione ruotata
    # del vettore accelerazione sui tre assi x, y, z
    acc_abs = np.transpose(J(roll, pitch, yaw))@ np.transpose(accelerations[i,:]) - np.array([[0, 0, 9.96]]).T
    acc_global.append((acc_abs[0,0], acc_abs[1,0], acc_abs[2,0]))
    


print(quat)
print(cos(0))

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



plt.show()
