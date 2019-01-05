import cv2
import os
import numpy as np
from pyquaternion import Quaternion

import coord_transforms
import align_timestamps

import scipy.io
import math

# data_path = '../../data/quad3/'
# save_path = './basic-reg-saves/70/'
# rgb_path = data_path + 'rgb/'
# depth_path = data_path + 'depth/'

# rgb_ims, depth_ims = align_timestamps.get_names(10)

data_path = '../../data/vicon/without/'
save_path = data_path
rgb_path = data_path + 'rgb/'
depth_path = data_path + 'depth/'
rgb_ims = ['1455208372.86.png']
depth_ims = ['1455208372.86.png']

print(len(rgb_ims))

for i in range(len(rgb_ims)):
	ts = rgb_ims[i][:-4]
	if len(ts)<len('1533793678.72'):
		ts += '0'
	fo = open(data_path + 'pcs/' + ts +'.ply','w')
	
	# print(rgb_path + rgb_ims[i])
	img = cv2.imread(rgb_path + rgb_ims[i])
	dimg = cv2.imread(depth_path + depth_ims[i],0)


	P = []
	C = []
	for j in range(img.shape[0]):
		for k in range(img.shape[1]): 
			#convert to camera frame 1 first
			d = dimg[j,k]

			if d > 0:
				P += [coord_transforms.im_to_cam(k, j, d)]

				C += [img[j, k,:]]
	
	P = np.vstack(P)
	C = np.vstack(C)

	fo.write('ply\n')
	fo.write('format ascii 1.0\n')
	fo.write('element vertex ' + str(P.shape[0]) + '\n')
	fo.write('property float x\nproperty float y\nproperty float z\n')
	fo.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
	for j in range(len(P)):
		fo.write(str(P[j,0])+' '+str(P[j,1])+' '+str(P[j,2])+' '+str(C[j,0])+' '+str(C[j,1])+' '+str(C[j,2])+'\n')

	fo.close()