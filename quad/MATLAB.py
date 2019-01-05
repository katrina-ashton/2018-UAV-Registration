import numpy as np
import scipy.io
import os
from pyquaternion import Quaternion

from eval import save_plot, compare, process_comp, align_gt
import align_timestamps

step = 5

data_path = '../../data/quad3/'
save_root = './basic-reg-saves/'

rgb_path = data_path + 'rgb/'
depth_path = data_path + 'depth/'
rgb_ims = os.listdir(rgb_path)
depth_ims = os.listdir(depth_path)

rgb_ims = align_timestamps.sample_rgb(rgb_ims, step*10)

rgb_ts = [x[:-4] for x in rgb_ims]
rgb_ts = np.vstack(rgb_ts).astype(np.float)

fo  = open(data_path + 'groundtruth.txt', 'r')
fo.readline()
# print(fo.readline())
trj_gt = []
qtrj_gt = []
t_gt = []
for line in fo:
	ele = line[:-1].split(' ')
	if len(ele)==14:
		rotm = align_timestamps.eulerAnglesToRotationMatrix([float(ele[1]),float(ele[2]),float(ele[3])])
		q = Quaternion(matrix=rotm)
		t_gt += [float(ele[0])]
		trj_gt += [[ float(ele[7]), float(ele[8]), float(ele[9])]]
		qtrj_gt += [q]
t_gt = np.vstack(t_gt)
trj_gt = np.vstack(trj_gt)#.astype(np.float)
qtrj_gt = np.vstack(qtrj_gt)

trj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, trj_gt)
qtrj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, qtrj_gt)

trj_gt, qtrj_gt = align_gt(trj_gt,qtrj_gt)

rot_gt = np.dstack([x.rotation_matrix for x in qtrj_gt[:,0]])

rot_icp = scipy.io.loadmat('MATLAB/Rs-'+str(step)+'.mat')['Rs']
trj_icp = scipy.io.loadmat('MATLAB/ts-'+str(step)+'.mat')['ts']

# print(rot_icp.shape)
print(trj_icp)

save_path = save_root + str(step*10) + '/'
lim = 1.5
s = 0.1
save_plot(rot_icp,trj_icp,lim,s,'Trajectory estimated using ICP','atrj_icp',save_path)
# save_plot(rot_gt,trj_gt,lim,s,'Ground truth trajectory','atrj_gt',save_path)

et_icp, eR_icp = compare(rot_gt,trj_gt,rot_icp,trj_icp)

process_comp(rgb_ts,et_icp,eR_icp,'a','icp','ICP',save_path)

rrot_icp = scipy.io.loadmat('MATLAB/Rsr-'+str(step)+'.mat')['Rsr']
rtrj_icp = scipy.io.loadmat('MATLAB/tsr-'+str(step)+'.mat')['tsr']

for i in range(rrot_icp.shape[2]):
	rrot_icp[:,:,i] = np.dot(rot_gt[:,:,i], rrot_icp[:,:,i])
	rtrj_icp[i,:] = trj_gt[i,:] + np.dot(rot_gt[:,:,i],rtrj_icp[i,:].transpose()).transpose()


save_plot(rrot_icp,rtrj_icp,lim,s,'Trajectory estimated using ICP','rtrj_icp',save_path)

ret_icp, reR_icp = compare(rot_gt,trj_gt,rrot_icp,rtrj_icp)

process_comp(rgb_ts,ret_icp,reR_icp,'r','icp','ICP',save_path)

tm = scipy.io.loadmat('MATLAB/tm-'+str(step)+'.mat')['tm']
print(tm)
