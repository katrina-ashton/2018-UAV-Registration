import cv2
import pyquaternion
import sys
import numpy as np
import matplotlib as plt 

# print(sys.version)
# print(cv2.__version__)
# print(np.__version__)
# print(plt.__version__)


skip = 20
save_root = './basic-reg-saves-new-rectangle-2/'

trj = np.load(save_root + str(skip) + '/qtrj_pnp.npy')
n = len(trj)
# print(n)

t = np.load(save_root + str(skip) + '/tpnp' + '.npy')
# print(t)
# print(np.sum(t))
print(np.sum(t)/n)