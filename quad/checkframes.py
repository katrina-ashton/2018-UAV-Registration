import numpy as np
import matplotlib.pyplot as plt 
from pyquaternion import Quaternion
from mpl_toolkits.mplot3d import Axes3D

import align_timestamps

import math

posx = np.array([1,0,0])
posy = np.array([0,1,0])
posz = np.array([0,0,1])

R1 = align_timestamps.Rx(math.pi/4+math.pi/2)
R2 = align_timestamps.Rz(math.pi/2) 
# R3 = align_timestamps.Rx(math.pi) 


#Upright: posx = [1,0,0]; posy = [0,sqrt(0.5),sqrt(0.5)]; posz = [0,-sqrt(0.5),sqrt(0.5)]
#Quad: posx = [0,1,0]; posy = [-sqrt(0.5),0,sqrt(0.5)]; posz = [sqrt(0.5),0,sqrt(0.5)]
#World: posx = [0,-1,0]; posy = [-sqrt(0.5),0,-sqrt(0.5)]; posz = [sqrt(0.5),0,-sqrt(0.5)]
posx = np.dot(R1,posx.transpose()).transpose()
posy = np.dot(R1,posy.transpose()).transpose()
posz = np.dot(R1,posz.transpose()).transpose()
print(posx)
print(posy)
print(posz)
#yes

posx = np.dot(R2,posx.transpose()).transpose()
posy = np.dot(R2,posy.transpose()).transpose()
posz = np.dot(R2,posz.transpose()).transpose()
print(posx)
print(posy)
print(posz)
#yes

# posx = np.dot(R3,posx.transpose()).transpose()
# posy = np.dot(R3,posy.transpose()).transpose()
# posz = np.dot(R3,posz.transpose()).transpose()
# print(posx)
# print(posy)
# print(posz)



# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(pos[0],pos[1],pos[2],'*')
# plt.show()