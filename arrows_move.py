import numpy as np
import random as rd
from matplotlib import pyplot as plt, animation
import pandas as pd

"""
Code for arrows that move whit the output of Madgwick
function
"""


with open('orientazioni_0.txt', encoding='utf-8') as f:
    file=f.readline()
    listaData = []
    while file != '':
        lst = file.split(',')
        #print(lst[2])
        listaData.append(lst[2])
        file=f.readline()

yaw = lst[2]
plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

x, y = np.mgrid[:2 * np.pi:10j, :2 * np.pi:5j]
u = np.cos(x)
v = np.sin(y)

fig, ax = plt.subplots(1, 1)
qr = ax.quiver(x, y, u, v, color='red')

def animate(num, qr, x, y, yaw):
   r = 2
   u = r* np.cos(yaw) #np.cos(x + num * 0.1) x = r × cos( θ ) y = r × sin( θ )
   v = r*np.sin(yaw)#np.sin(y + num * 0.1)
   qr.set_UVC(u, v)
   #qr.set_color((rd.random(), rd.random(), rd.random(), rd.random()))
   return qr,


for i in len(yaw):
   anim = animation.FuncAnimation(fig, animate, fargs=(qr, x, y, yaw[i]),
                              interval=50, blit=False)

   plt.show()