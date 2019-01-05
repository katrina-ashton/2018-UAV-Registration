import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
import math

def plot_axis(ax,R,t,s,lab):
	x = s*np.array([1,0,0])
	y = s*np.array([0,1,0])
	z = s*np.array([0,0,1])

	x = np.dot(R,x) + t
	y = np.dot(R,y) + t
	z = np.dot(R,z) + t

	ax.plot([t[0],x[0]],[t[1],x[1]],[t[2],x[2]], 'b')
	ax.plot([t[0],y[0]],[t[1],y[1]],[t[2],y[2]], 'g')
	ax.plot([t[0],z[0]],[t[1],z[1]],[t[2],z[2]], 'r')

	ax.text(t[0], t[1], t[2], lab)

	ax.legend(['x','y','z'])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# save_path = './basic-reg-saves/'
# qtrj_rgb = np.load(save_path + 'qtrj_rgb.npy')
# rot = qtrj_rgb[0][0].rotation_matrix
# plot_axis(ax, rot, np.array([0,0.5,0]))
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# plt.show()

