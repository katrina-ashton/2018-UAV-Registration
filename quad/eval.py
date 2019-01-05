import cv2
import os
import numpy as np
import matplotlib.pyplot as plt 
from pyquaternion import Quaternion
from mpl_toolkits.mplot3d import Axes3D

import align_timestamps
import plotting

import math


def get_rot(q, R0):
	# Rc2q = align_timestamps.Rc2q()
	# return np.dot(Rc2q, q.rotation_matrix)
	return q.rotation_matrix

def align_trj(trj, R0):
	# Rc2q = align_timestamps.Rc2q()

	# trj = np.dot(Rc2q,trj.transpose()).transpose()
	return trj

def align_gt(trj_gt, qtrj_gt):
	R1 = align_timestamps.Rx(math.pi)
	trj_gt = np.dot(R1,trj_gt.transpose()).transpose()
	return trj_gt, qtrj_gt


def get_trjs(data_path, save_path, step):
	rgb_path = data_path + 'rgb/'
	depth_path = data_path + 'depth/'
	rgb_ims = os.listdir(rgb_path)
	depth_ims = os.listdir(depth_path)

	rgb_ims = align_timestamps.sample_rgb(rgb_ims, step)

	rgb_ts = [x[:-4] for x in rgb_ims]
	rgb_ts = np.vstack(rgb_ts).astype(np.float)


	fo  = open(data_path + 'groundtruth.txt', 'r')
	fo.readline()
	trj_gt = []
	qtrj_gt = []
	t_gt = []
	for line in fo:
		ele = line[:-1].split(' ')
		if len(ele)==14:
			rotm = align_timestamps.eulerAnglesToRotationMatrix([float(ele[1]),\
				float(ele[2]),float(ele[3])])
			q = Quaternion(matrix=rotm)
			t_gt += [float(ele[0])]
			trj_gt += [[ float(ele[7]), float(ele[8]), float(ele[9])]]
			qtrj_gt += [q]
	t_gt = np.vstack(t_gt)
	trj_gt = np.vstack(trj_gt)#.astype(np.float)
	qtrj_gt = np.vstack(qtrj_gt)

	suff = ''
	trj_rgb = np.load(save_path + 'trj_rgb' + suff + '.npy')
	qtrj_rgb = np.load(save_path + 'qtrj_rgb' + suff + '.npy')
	trj_d = np.load(save_path + 'trj_d' + suff + '.npy')
	qtrj_d = np.load(save_path + 'qtrj_d' + suff + '.npy')
	trj_pnp = np.load(save_path + 'trj_pnp' + suff + '.npy')
	qtrj_pnp = np.load(save_path + 'qtrj_pnp' + suff + '.npy')
	trj_pnpe = np.load(save_path + 'trj_pnpe' + suff + '.npy')
	qtrj_pnpe = np.load(save_path + 'qtrj_pnpe' + suff + '.npy')
	trj_de = np.load(save_path + 'trj_de' + suff + '.npy')
	qtrj_de = np.load(save_path + 'qtrj_de' + suff + '.npy')
	trj_dp = np.load(save_path + 'trj_dp' + suff + '.npy')
	qtrj_dp = np.load(save_path + 'qtrj_dp' + suff + '.npy')

	trj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, trj_gt)
	qtrj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, qtrj_gt)

	trj_gt, qtrj_gt = align_gt(trj_gt,qtrj_gt)

	trj_rgb = align_trj(trj_rgb, qtrj_gt[0,0].rotation_matrix)
	trj_d = align_trj(trj_d, qtrj_gt[0,0].rotation_matrix)
	trj_de = align_trj(trj_de, qtrj_gt[0,0].rotation_matrix)
	trj_dp = align_trj(trj_dp, qtrj_gt[0,0].rotation_matrix)
	trj_pnp = align_trj(trj_pnp, qtrj_gt[0,0].rotation_matrix)
	trj_pnpe = align_trj(trj_pnpe, qtrj_gt[0,0].rotation_matrix)

	return trj_gt, qtrj_gt, trj_rgb, qtrj_rgb, trj_d, qtrj_d, trj_pnp, qtrj_pnp, \
		trj_de, qtrj_de, trj_dp, qtrj_dp, trj_pnpe, qtrj_pnpe, rgb_ts

def get_rtrjs(data_path, save_path, step):
	rgb_path = data_path + 'rgb/'
	depth_path = data_path + 'depth/'
	rgb_ims = os.listdir(rgb_path)
	depth_ims = os.listdir(depth_path)

	rgb_ims = align_timestamps.sample_rgb(rgb_ims,step)

	rgb_ts = [x[:-4] for x in rgb_ims]
	rgb_ts = np.vstack(rgb_ts).astype(np.float)


	fo  = open(data_path + 'groundtruth.txt', 'r')
	fo.readline()
	trj_gt = []
	qtrj_gt = []
	t_gt = []
	for line in fo:
		ele = line[:-1].split(' ')
		if len(ele)==14:
			rotm = align_timestamps.eulerAnglesToRotationMatrix([float(ele[1]),\
				float(ele[2]),float(ele[3])])
			q = Quaternion(matrix=rotm)
			t_gt += [float(ele[0])]
			trj_gt += [[ float(ele[7]), float(ele[8]), float(ele[9])]]
			qtrj_gt += [q]
	t_gt = np.vstack(t_gt)
	trj_gt = np.vstack(trj_gt)
	qtrj_gt = np.vstack(qtrj_gt)

	suff = ''
	trj_rgb = np.load(save_path + 'rtrj_rgb' + suff + '.npy')
	qtrj_rgb = np.load(save_path + 'rqtrj_rgb' + suff + '.npy')
	trj_d = np.load(save_path + 'rtrj_d' + suff + '.npy')
	qtrj_d = np.load(save_path + 'rqtrj_d' + suff + '.npy')
	trj_pnp = np.load(save_path + 'rtrj_pnp' + suff + '.npy')
	qtrj_pnp = np.load(save_path + 'rqtrj_pnp' + suff + '.npy')
	trj_pnpe = np.load(save_path + 'rtrj_pnpe' + suff + '.npy')
	qtrj_pnpe = np.load(save_path + 'rqtrj_pnpe' + suff + '.npy')
	trj_de = np.load(save_path + 'rtrj_de' + suff + '.npy')
	qtrj_de = np.load(save_path + 'rqtrj_de' + suff + '.npy')
	trj_dp = np.load(save_path + 'rtrj_dp' + suff + '.npy')
	qtrj_dp = np.load(save_path + 'rqtrj_dp' + suff + '.npy')

	trj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, trj_gt)
	qtrj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, qtrj_gt)

	trj_gt, qtrj_gt = align_gt(trj_gt,qtrj_gt)

	trj_rgb = align_trj(trj_rgb, qtrj_gt[0,0].rotation_matrix)
	trj_d = align_trj(trj_d, qtrj_gt[0,0].rotation_matrix)
	trj_de = align_trj(trj_de, qtrj_gt[0,0].rotation_matrix)
	trj_dp = align_trj(trj_dp, qtrj_gt[0,0].rotation_matrix)
	trj_pnp = align_trj(trj_pnp, qtrj_gt[0,0].rotation_matrix)
	trj_pnpe = align_trj(trj_pnpe, qtrj_gt[0,0].rotation_matrix)

	return trj_gt, qtrj_gt, trj_rgb, qtrj_rgb, trj_d, qtrj_d, trj_pnp, qtrj_pnp, \
		trj_de, qtrj_de, trj_dp, qtrj_dp, trj_pnpe, qtrj_pnpe, rgb_ts

def get_rotm(qtrj_gt, qtrj_rgb, qtrj_d, qtrj_pnp, qtrj_de, qtrj_dp, qtrj_pnpe):

	rot_gt = np.dstack([x.rotation_matrix for x in qtrj_gt[:,0]])
	rot_rgb = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_rgb[:,0]])
	rot_d = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_d[:,0]])
	rot_pnp = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_pnp[:,0]])
	rot_pnpe = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_pnpe[:,0]])
	rot_de = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_de[:,0]])
	rot_dp = np.dstack([get_rot(x, qtrj_gt[0,0].rotation_matrix) for x in qtrj_dp[:,0]])

	return rot_gt, rot_rgb, rot_d, rot_pnp, rot_de, rot_dp, rot_pnpe


def save_plot(rot,pos,lim,s,title,name,save_path):
	f = plt.figure()
	ax = f.add_subplot(111, projection='3d')
	for i in range(pos.shape[0]):
		plotting.plot_axis(ax,rot[:,:,i],pos[i,:],s, i)
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.set_title(title)
	ax.set_xlim(-lim,lim)
	ax.set_ylim(-lim,lim)
	ax.set_zlim(-lim,lim)
	f.savefig(save_path + name + '.pdf', bbox_inches='tight')
	plt.close()

def compare(rot_gt,pos_gt,rot,pos):
	#get transform from gt to estimated:

	if len(rot.shape) < 3:
		t = -pos + pos_gt
		et = np.sqrt(np.sum(np.square(t)))
		R = np.dot(rot_gt[:,:],rot[:,:].transpose())
		q = Quaternion(matrix=R)
		ar = q.radians
		ar = ar%math.pi
		eR = ar
	else:
		t = -pos + pos_gt
		et = np.sqrt(np.sum(np.square(t),axis=1))
		eR = []
		for i in range(rot.shape[2]):
			R = np.dot(rot_gt[:,:,i],rot[:,:,i].transpose())
			q = Quaternion(matrix=R)
			ar = q.radians
			ar = ar%math.pi
			eR += [ar]

	return et, eR

def process_comp(ts, et,eR,pre,suff,name,save_path):
	f = plt.figure()
	plt.plot(ts, et)
	plt.title('Translation error for trajectory estimated using ' + name)
	plt.xlabel('Time')
	plt.ylabel('Eulidean distance')
	f.savefig(save_path + pre + 'et_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

	f = plt.figure()
	plt.plot(ts, eR)
	plt.title('Rotation error for trajectory estimated using ' + name)
	plt.xlabel('Time')
	plt.ylabel('Magnitude of rotation')
	f.savefig(save_path + pre + 'eR_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

	print(name, np.sum(et), np.sum(eR))

def process_comp_all(ts, ets,eRs,save_path,pre=''):
	f = plt.figure()
	for i in range(ets.shape[0]):
		plt.plot(ts, ets[i,:])
	plt.title('Translation error for all estimation methods')
	plt.xlabel('Time')
	plt.ylabel('Eulidean distance (meters)')
	plt.legend(['Essential Matrix', 'Kabsch', 'Kabsch (EM inliers)', \
		'Kabsch (PnP inliers)', 'iterative PnP', 'EPnP'])
	f.savefig(save_path + pre + 'et_all' + '.pdf', bbox_inches='tight')
	plt.close()

	f = plt.figure()
	for i in range(eRs.shape[0]):
		plt.plot(ts, eRs[i,:])
	plt.title('Rotation error for all estimation methods')
	plt.xlabel('Time')
	plt.ylabel('Magnitude of rotation (radians)')
	plt.legend(['Essential Matrix', 'Kabsch', 'Kabsch (EM inliers)', \
		'Kabsch (PnP inliers)', 'iterative PnP', 'EPnP'])
	f.savefig(save_path + pre + 'eR_all' + '.pdf', bbox_inches='tight')
	plt.close()


def process_comp_method(ts, ets,eRs,save_path,name,suff,pre=''):
	s = 'skip'
	f = plt.figure()
	for i in range(len(ets)):
		plt.plot(ts[i], ets[i])
	plt.title('Translation error for ' + name + ' method')
	plt.xlabel('Time')
	plt.ylabel('Eulidean distance (meters)')
	plt.legend([s+' 20',s+' 30',s+' 40',s+' 50',s+' 60',s+' 70',s+' 80',s+' 90'])
	# plt.savefig(save_path + pre + 'et_' + suff)
	f.savefig(save_path + pre + 'et_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

	f = plt.figure()
	for i in range(len(eRs)):
		plt.plot(ts[i], eRs[i])
	plt.title('Rotation error for ' + name + ' method')
	plt.xlabel('Time')
	plt.ylabel('Magnitude of rotation (radians)')
	plt.legend([s+' 20',s+' 30',s+' 40',s+' 50',s+' 60',s+' 70',s+' 80',s+' 90'])
	# plt.savefig(save_path + pre + 'eR_' + suff)
	f.savefig(save_path + pre + 'eR_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

	for i in range(len(ets)):
		print("& %d & %.4f & %.4f & %.4f & %.4f \\\\" % \
			((i+2)*10, np.mean(eRs[i]), np.std(eRs[i]), np.mean(ets[i]), np.std(ets[i])))

def process_comp_best(ts, idx, ets,eRs,save_path,names,suff,pre=''):
	f = plt.figure()
	for i in range(len(idx)):
		j = idx[i]
		plt.plot(ts[j], ets[i][j])
	plt.title('Translation error for each method, best skip')
	plt.xlabel('Time')
	plt.ylabel('Eulidean distance (meters)')
	plt.legend(names)
	f.savefig(save_path + pre + 'et_best_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

	f = plt.figure()
	for i in range(len(idx)):
		j = idx[i]
		plt.plot(ts[j], eRs[i][j])
	plt.title('Rotation error for each method, best skip')
	plt.xlabel('Time')
	plt.ylabel('Magnitude of rotation (radians)')
	plt.legend(names)
	f.savefig(save_path + pre + 'eR_best_' + suff + '.pdf', bbox_inches='tight')
	plt.close()

