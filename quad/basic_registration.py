import cv2
import os
import numpy as np
import matplotlib.pyplot as plt 
from pyquaternion import Quaternion
from mpl_toolkits.mplot3d import Axes3D
import time

import coord_transforms
import align_timestamps
import kabsch

import scipy.io
import math

#set RGB image folder path
data_path = '../../data/quad3/'
save_path = './basic-reg-saves/20/'
rgb_path = data_path + 'rgb/'
depth_path = data_path + 'depth/'

rgb_ims, depth_ims = align_timestamps.get_names(20)

rgb_scale= 7 #10 - 10 #20 - 7 #30 - 4.5 #40 - 4 #50 - 3 #60 - 2

# for im in rgb_ims:
# 	im = cv2.imread(rgb_path + im,0)
# 	plt.imshow(im)
# 	plt.show()

fo  = open(data_path + 'groundtruth.txt', 'r')
# fo.readline()
# fo.readline()
print(fo.readline())
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

rgb_ts = [x[:-4] for x in rgb_ims]
rgb_ts = np.vstack(rgb_ts).astype(np.float)

trj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, trj_gt)
qtrj_gt = align_timestamps.gt_with_rgb(t_gt, rgb_ts, qtrj_gt)


sp = trj_gt[0,:]
so = qtrj_gt[0][0]

print(sp)

# R1 = align_timestamps.Rx(math.pi/4)
# R2 = align_timestamps.Rz(math.pi/2) 
# R = np.dot(R2,R1).transpose()
R = align_timestamps.Rx(math.pi)
sp = np.dot(R,sp.transpose()).transpose()

print(sp)
print(so.rotation_matrix)

R1 = align_timestamps.Rx(math.pi/2 + math.pi/4)
R2 = align_timestamps.Rz(math.pi/2) 
R3 = align_timestamps.Rx(math.pi) 
Rq2w = so.rotation_matrix.transpose()
Rc2q = np.dot(R2,R1)


# sp = np.array([0,0,0])
# so = np.eye(3)
# so = Quaternion(matrix=so) 

pos_rgb = sp
pos_d = sp
pos_pnp = sp
pos_pnpe = sp
pos_de = sp
pos_dp = sp

trj_rgb = [pos_rgb]
trj_d = [pos_d]
trj_pnp = [pos_pnp]
trj_pnpe = [pos_pnpe]
trj_de = [pos_de]
trj_dp = [pos_dp]

qtrj_rgb = [so]
qtrj_d = [so]
qtrj_pnp = [so]
qtrj_pnpe = [so]
qtrj_de = [so]
qtrj_dp = [so]

rtrj_rgb = [pos_rgb]
rtrj_d = [pos_d]
rtrj_pnp = [pos_pnp]
rtrj_pnpe = [pos_pnpe]
rtrj_de = [pos_de]
rtrj_dp = [pos_dp]

rqtrj_rgb = [so]
rqtrj_d = [so]
rqtrj_pnp = [so]
rqtrj_pnpe = [so]
rqtrj_de = [so]
rqtrj_dp = [so]

so_c = np.dot(Rc2q.transpose(),so.rotation_matrix)

rot_rgb = so_c
rot_d = so_c
rot_pnp = so_c
rot_pnpe = so_c
rot_de = so_c
rot_dp = so_c

mat = np.eye(3)

good_i = []

# plt.ion()
# fig = plt.figure()
# fig.show()
# fig.legend(['rgb', 'depth'])
# ax = fig.add_subplot(111)

fx = 616.9660034179688
fy = 616.8399047851562
cx = 328.9248962402344 
cy = 230.74755859375
focal = np.sqrt(fx**2+fy**2)
pp = (cx,cy)

cmat = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])

tm_rgb = 0
tm_d = 0
tm_de = 0
tm_dp = 0
tm_pnp = 0
tm_pnpe = 0
tm_fd = 0
tm_ff = 0

p = 0.9999

for i in range(len(rgb_ims)-1):
	# print(rgb_path + rgb_ims[i])
	img1 = cv2.imread(rgb_path + rgb_ims[i],0)          # queryImage
	img2 = cv2.imread(rgb_path + rgb_ims[i+1],0) # trainImage

	# print(img1.shape)

	mask = np.array(img1.shape, dtype=np.uint8)


	t = time.time()
	# Initiate SIFT detector
	orb = cv2.xfeatures2d.SIFT_create()

	# find the keypoints and descriptors with SIFT - image frame
	kp1, des1 = orb.detectAndCompute(img1,None)
	kp2, des2 = orb.detectAndCompute(img2,None)

	# print(np.max(des1))

	des1 = des1.astype(np.uint8)
	des2 = des2.astype(np.uint8)


	# create BFMatcher object
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

	# Match descriptors.
	matches = bf.match(des1,des2) #train, query ?

	imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches, None)
	plt.imshow(imgm)
	plt.savefig('matches/{}_before.png'.format(rgb_ims[i][:-4]))
	plt.clf()


	# Sort them in the order of their distance.
	matches = sorted(matches, key = lambda x:x.distance)

	#Get points
	pt1 = []
	pt2 = []

	flag = 0
	ng = 0
	for j in range(len(matches)-1):
	# 	# print(matches[j].distance)
	# 	
		pt1 += [kp1[matches[j].queryIdx].pt]
		pt2 += [kp2[matches[j].trainIdx].pt]
	# 	if matches[j].distance < 180:
	# 		ng+= 1
	# 	else:
	# 		if ng <6:
	# 			flag =1
	# 			print('bad')
	# 			break	
	# if flag==1:
	# 	continue	
	good_i += [i]	
	# print(good_i)	

	pt1 = np.vstack(pt1)
	pt2 = np.vstack(pt2)

	# print(np.max(pt1,axis=0))

	tm_ff += time.time() - t

	#with Essential Matrix:
	t = time.time()
	E = cv2.findEssentialMat(pt2, pt1, focal, pp, cv2.RANSAC, p, 1.0, None)

	#bad - no change in position/orientation
	# if sum(E[1]) < 6:
	# 	trj_rgb += [trj_rgb[-1]]
	# 	qtrj_rgb += [qtrj_rgb[-1]]

	# 	trj_d += [trj_d[-1]]
	# 	qtrj_d += [qtrj_d[-1]]		

	# 	continue

	# bm = np.array(np.concatenate((E[1],E[1]),axis=1), dtype=bool)
	# pt1m = pt1[bm]
	# pt1 = np.concatenate((np.vstack(pt1m)[0:-2:2], np.vstack(pt1m)[1:-1:2]),axis=1)
	# pt2m = pt2[bm]
	# pt2 = np.concatenate((np.vstack(pt2m)[0:-2:2], np.vstack(pt2m)[1:-1:2]),axis=1)
	
	R_rgb = np.zeros((3,3))
	t_rgb = np.zeros((3,1))
	cv2.recoverPose(E[0], pt2, pt1, R_rgb, t_rgb, focal, pp, None) #camera frame 1??

	tm_rgb += time.time() - t

	#visulaize
	matches_good = []
	for m in range(len(E[1])):
		if E[1][m] == 1:
			matches_good += [matches[m]]

	imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
	plt.imshow(imgm)
	plt.savefig('matches/{}_essential.png'.format(rgb_ims[i][:-4]))
	plt.clf()

	#with Kabsch:
	t = time.time()
	d_img1 = cv2.imread(depth_path + depth_ims[i],0)   # queryImage
	d_img2 = cv2.imread(depth_path + depth_ims[i+1],0) # trainImage

	# plt.imshow(d_img1)
	# plt.show()
	# plt.imshow(d_img2)
	# plt.show()


	pt1 = pt1.astype(np.uint32)
	pt2 = pt2.astype(np.uint32)

	pt1f = []
	pt2f = []

	P = []
	Q = []

	Pe = []
	Qe = []

	rgb_img1 = cv2.imread(rgb_path + rgb_ims[i])          # queryImage
	rgb_img2 = cv2.imread(rgb_path + rgb_ims[i+1]) # trainImage
	Cp = []
	Cq = []

	matches3 = []

	for j in range(pt1.shape[0]):
		#convert to camera frame 1 first
		d1 = d_img1[pt1[j,1], pt1[j,0]]
		d2 = d_img2[pt2[j,1], pt2[j,0]]

		if d1 > 0 and d2 >0:
			P += [coord_transforms.im_to_cam(pt1[j,0], pt1[j,1], d1)]
			Q += [coord_transforms.im_to_cam(pt2[j,0], pt2[j,1], d2)]

			Cp += [rgb_img1[pt1[j,1], pt1[j,0],:]]
			Cq += [rgb_img2[pt1[j,1], pt1[j,0],:]]

			pt1f += [[float(pt1[j,0]), float(pt1[j,1])]]
			pt2f += [[float(pt2[j,0]), float(pt2[j,1])]]

			matches3 += [matches[j]]

			if j in E[1][0]:
				Pe += [coord_transforms.im_to_cam(pt1[j,0], pt1[j,1], d1)]
				Qe += [coord_transforms.im_to_cam(pt2[j,0], pt2[j,1], d2)]


	pt1f = np.vstack(pt1f)
	pt2f = np.vstack(pt2f)
	P = np.vstack(P)
	Q = np.vstack(Q)
	Cp = np.vstack(Cp)/255.0
	Cq = np.vstack(Cq)/255.0
	Pe = np.vstack(P)
	Qe = np.vstack(Q)

	tm_fd += time.time() - t

	t = time.time()
	R_d, t_d, inliers = kabsch.kabsch(P, Q, p=p, v=0.5, dist_thresh = 0.1)
	tm_d += time.time() - t
	
	matches_good = []
	for m in inliers:
		matches_good += [matches3[m]]

	imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
	plt.imshow(imgm)
	plt.savefig('matches/{}_kabsch.png'.format(rgb_ims[i][:-4]))
	plt.clf()
	
	# R_d = R_d.transpose()

	# P = P + np.repeat(p0,P.shape[0],axis=0).astype(np.float32)
	# Q = Q + np.repeat(q0,Q.shape[0],axis=0).astype(np.float32)

	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(P[:,0], P[:,1], P[:,2], c=Cp)
	# ax.set_xlabel('X')
	# ax.set_ylabel('Y')
	# ax.set_zlabel('Z')
	# # plt.show()
	# plt.savefig('PCs/{}_P.png'.format(rgb_ims[i][:-4]))
	# plt.clf()

	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(Q[:,0], Q[:,1], Q[:,2], c=Cq)
	# ax.set_xlabel('X')
	# ax.set_ylabel('Y')
	# ax.set_zlabel('Z')
	# # plt.show()
	# plt.savefig('PCs/{}_Q.png'.format(rgb_ims[i][:-4]))
	# plt.clf()

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(P[:,0], P[:,1], P[:,2], c='b')
	ax.scatter(Q[:,0], Q[:,1], Q[:,2], c='g')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.legend(['P', 'Q'])
	# plt.show()
	plt.savefig('PCs/{}_PQ.png'.format(rgb_ims[i][:-4]))
	plt.clf()

	# Pa = np.zeros(P.shape)

	# for j in range(P.shape[0]):
	# 	Pa[j,:] = np.dot(R_d, P[j,:].transpose()) + t_d.transpose()

	Pa = np.dot(P, R_d.transpose()) + np.repeat(t_d.transpose(), P.shape[0], axis=0)

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(Pa[:,0], Pa[:,1], Pa[:,2], c='b')
	ax.scatter(Q[:,0], Q[:,1], Q[:,2], c='g')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.legend(['P aligned', 'Q'])
	# plt.show()
	plt.savefig('PCs/{}_PQ-aligned.png'.format(rgb_ims[i][:-4]))
	plt.clf()

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(P[inliers,0], P[inliers,1], P[inliers,2], c='b')
	ax.scatter(Q[inliers,0], Q[inliers,1], Q[inliers,2], c='g')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.legend(['P', 'Q'])
	# plt.show()
	plt.savefig('PCs/{}_PQ-inliers.png'.format(rgb_ims[i][:-4]))
	plt.clf()

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.scatter(Pa[inliers,0], Pa[inliers,1], Pa[inliers,2], c='b')
	ax.scatter(Q[inliers,0], Q[inliers,1], Q[inliers,2], c='g')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	ax.legend(['P aligned', 'Q'])
	# plt.show()
	plt.savefig('PCs/{}_PQ-aligned-inliers.png'.format(rgb_ims[i][:-4]))
	plt.clf()

	# Pa = np.zeros(P.shape)

	# for j in range(P.shape[0]):
	# 	Pa[j,:] = np.dot(R_d, P[j,:].transpose()) - t_d.transpose()

	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(Pa[:,0], Pa[:,1], Pa[:,2], c='b')
	# ax.scatter(Q[:,0], Q[:,1], Q[:,2], c='g')
	# ax.set_xlabel('X')
	# ax.set_ylabel('Y')
	# ax.set_zlabel('Z')
	# ax.legend(['P aligned', 'Q'])
	# # plt.show()
	# plt.savefig('PCs/{}_PQ-aligned-invt.png'.format(rgb_ims[i][:-4]))
	# plt.clf()

	# Pa = np.zeros(P.shape)

	# for j in range(P.shape[0]):
	# 	Pa[j,:] = np.dot(R_d, P[j,:].transpose())

	# fig = plt.figure()
	# ax = fig.add_subplot(111, projection='3d')
	# ax.scatter(Pa[:,0], Pa[:,1], Pa[:,2], c='b')
	# ax.scatter(Q[:,0], Q[:,1], Q[:,2], c='g')
	# ax.set_xlabel('X')
	# ax.set_ylabel('Y')
	# ax.set_zlabel('Z')
	# ax.legend(['P aligned', 'Q'])
	# # plt.show()
	# plt.savefig('PCs/{}_PQ-aligned-no.png'.format(rgb_ims[i][:-4]))
	# plt.clf()

	#with PnP:
	v = 0.8
	m = 5
	N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))
	dist_thresh = 1 #0.05

	P = P.reshape((P.shape[0],1,3))
	pt2f = pt2f.reshape((pt2f.shape[0],1,2))

	t = time.time()
	_, R_vec, t_pnp, in_pnp = cv2.solvePnPRansac(P, pt2f, cmat, None, iterationsCount=N, reprojectionError=dist_thresh, confidence=p, flags=cv2.SOLVEPNP_ITERATIVE)
	# _, R_vec, t_pnp = cv2.solvePnP(P, pt2f, cmat, None, flags=cv2.SOLVEPNP_EPNP)
	tm_pnp += time.time() - t

	R_pnp = np.zeros((3,3))
	cv2.Rodrigues(R_vec,R_pnp)


	#visulaize
	matches_good = []
	if in_pnp is not None:
		for m in in_pnp:
			matches_good += [matches3[m[0]]]

		imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
		plt.imshow(imgm)
		plt.savefig('matches/{}_pnp.png'.format(rgb_ims[i][:-4]))
		plt.clf()

	t = time.time()
	_, R_vec, t_pnpe, _ = cv2.solvePnPRansac(P, pt2f, cmat, None, iterationsCount=N, reprojectionError=dist_thresh, confidence=p, flags=cv2.SOLVEPNP_EPNP)
	# _, R_vec, t_pnp = cv2.solvePnP(P, pt2f, cmat, None, flags=cv2.SOLVEPNP_EPNP)
	tm_pnpe += time.time() - t

	R_pnpe = np.zeros((3,3))
	cv2.Rodrigues(R_vec,R_pnpe)



	#Kabsch with inliers from EM, PnP
	t = time.time()
	R_de, t_de= kabsch.find_transform(Pe, Qe)
	tm_de += time.time() - t

	scipy.io.savemat('kabsch/e-{}.mat'.format(i), dict(P=Pe, Q=Qe, R=R_de, t=t_de))

	if in_pnp is None:
		R_dp = np.eye(3)
		t_d = np.array([0,0,0])
	else:
		Pi = P[in_pnp[:],:]
		Qi = Q[in_pnp[:],:]
		Pi = Pi.reshape((Pi.shape[0],3))
		Qi = Qi.reshape((Qi.shape[0],3))
		# print(i)
		t = time.time()
		R_dp, t_dp= kabsch.find_transform(Pi, Qi)
		tm_dp += time.time() - t

	scipy.io.savemat('kabsch/pnp-{}.mat'.format(i), dict(P=Pi, Q=Qi, R=R_dp, t=t_dp))


	#Update trajectories
	R_rgb = np.transpose(R_rgb)
	t_rgb = -t_rgb/rgb_scale

	# R_d = np.transpose(R_d)
	# t_d = -t_d
	# R_de = np.transpose(R_de)
	# t_de = -t_de

	# R_pnp = np.transpose(R_pnp)
	# t_pnp = -t_pnp
	# R_pnpe = np.transpose(R_pnpe)
	# t_pnpe = -t_pnpe

	# print(pos_rgb)
	# print(t_rgb)
	# print(np.dot(R1,t_rgb))

	# R_rgb = np.dot(Rc2q.transpose(),np.dot(R_rgb,Rc2q.transpose()))
	# t_rgb = np.dot(Rc2q,t_rgb)
	
	# R_d = np.dot(Rc2q,np.dot(R_d,Rc2q.transpose()))
	# t_d = np.dot(Rc2q,t_d)
	# R_de = np.dot(Rc2q,np.dot(R_de,Rc2q.transpose()))
	# t_de = np.dot(Rc2q,t_de)
	# R_dp = np.dot(Rc2q,np.dot(R_dp,Rc2q.transpose()))
	# t_dp = np.dot(Rc2q,t_dp)

	# R_pnp = np.dot(Rc2q,np.dot(R_pnp,Rc2q.transpose()))
	# t_pnp = np.dot(Rc2q,t_pnp)
	# R_pnpe = np.dot(Rc2q,np.dot(R_pnpe,Rc2q.transpose()))
	# t_pnpe = np.dot(Rc2q,t_pnpe)	

	Rq = np.dot(Rc2q.transpose(),qtrj_gt[i][0].rotation_matrix)
	pq = np.dot(R,trj_gt[i,:]).transpose()


	q_c = np.dot(rot_rgb, R_rgb)
	pos_rgb = pos_rgb + np.transpose(np.dot(rot_rgb.transpose(),t_rgb)) #first one is in global frame, so don't rotate
	rot_rgb = q_c

	trj_rgb += [pos_rgb]
	qtrj_rgb += [Quaternion(matrix=rot_rgb)]
	
	qr = np.dot(Rq, R_rgb)
	rpos_rgb = pq + np.transpose(Rq.transpose(),t_rgb)
	rtrj_rgb += [rpos_rgb]
	rqtrj_rgb += [Quaternion(matrix=qr)]	

	q_c = np.dot(rot_d, R_d)
	pos_d = pos_d + np.transpose(np.dot(rot_d.transpose(),t_d)) #t_d.reshape((3,)) #q_c.rotate(t_d)
	rot_d = q_c

	trj_d += [pos_d]
	qtrj_d += [Quaternion(matrix=rot_d)]

	qr = np.dot(Rq, R_d)
	rpos_d = pq + np.transpose(np.dot(Rq.transpose(),t_d))
	rtrj_d += [rpos_d]
	rqtrj_d += [Quaternion(matrix=qr)]	


	q_c = np.dot(rot_pnp, R_pnp)
	pos_pnp = pos_pnp + np.transpose(np.dot(rot_pnp.transpose(),t_pnp)) #t_d.reshape((3,)) #q_c.rotate(t_d)
	rot_pnp = q_c

	trj_pnp += [pos_pnp]
	qtrj_pnp += [Quaternion(matrix=rot_pnp)]

	qr = np.dot(Rq, R_pnp)
	rpos_pnp = pq + np.transpose(np.dot(Rq.transpose(),t_pnp))
	rtrj_pnp += [rpos_pnp]
	rqtrj_pnp += [Quaternion(matrix=qr)]	


	q_c = np.dot(rot_pnpe, R_pnpe)
	pos_pnpe = pos_pnpe + np.transpose(np.dot(rot_pnpe.transpose(),t_pnpe)) #t_d.reshape((3,)) #q_c.rotate(t_d)
	rot_pnpe = q_c

	trj_pnpe += [pos_pnpe]
	qtrj_pnpe += [Quaternion(matrix=rot_pnpe)]

	qr = np.dot(Rq, R_pnpe)
	rpos_pnpe = pq + np.transpose(np.dot(Rq.transpose(),t_pnpe))
	rtrj_pnpe += [rpos_pnpe]
	rqtrj_pnpe += [Quaternion(matrix=qr)]


	q_c = np.dot(rot_de, R_de)
	pos_de = pos_de + np.transpose(np.dot(rot_de.transpose(),t_de))#t_d.reshape((3,)) #q_c.rotate(t_d)
	rot_de = q_c

	trj_de += [pos_de]
	qtrj_de += [Quaternion(matrix=rot_de)]

	qr = np.dot(Rq, R_de)
	rpos_de = pq + np.transpose(np.dot(Rq.transpose(),t_de)) 
	rtrj_de += [rpos_de]
	rqtrj_de += [Quaternion(matrix=qr)]	


	q_c = np.dot(rot_de, R_dp)
	pos_dp = pos_dp + np.transpose(np.dot(rot_dp.transpose(),t_dp)) #t_d.reshape((3,)) #q_c.rotate(t_d)
	rot_dp = q_c

	trj_dp += [pos_de]
	qtrj_dp += [Quaternion(matrix=rot_dp)]

	qr = np.dot(Rq, R_dp)
	rpos_dp = pq + np.transpose(np.dot(Rq.transpose(),t_dp))
	rtrj_dp += [rpos_dp]
	rqtrj_dp += [Quaternion(matrix=qr)]	

trj_rgb = np.vstack(trj_rgb)
trj_d = np.vstack(trj_d)
trj_pnp = np.vstack(trj_pnp)
trj_pnpe = np.vstack(trj_pnpe)
trj_de = np.vstack(trj_de)
trj_dp = np.vstack(trj_dp)

qtrj_rgb = np.vstack(qtrj_rgb)
qtrj_d = np.vstack(qtrj_d)
qtrj_pnp = np.vstack(qtrj_pnp)
qtrj_pnpe = np.vstack(qtrj_pnpe)
qtrj_de = np.vstack(qtrj_de)
qtrj_dp = np.vstack(qtrj_dp)

rtrj_rgb = np.vstack(rtrj_rgb)
rtrj_d = np.vstack(rtrj_d)
rtrj_pnp = np.vstack(rtrj_pnp)
rtrj_pnpe = np.vstack(rtrj_pnpe)
rtrj_de = np.vstack(rtrj_de)
rtrj_dp = np.vstack(rtrj_dp)

rqtrj_rgb = np.vstack(rqtrj_rgb)
rqtrj_d = np.vstack(rqtrj_d)
rqtrj_pnp = np.vstack(rqtrj_pnp)
rqtrj_pnpe = np.vstack(rqtrj_pnpe)
rqtrj_de = np.vstack(rqtrj_de)
rqtrj_dp = np.vstack(rqtrj_dp)

np.save(save_path + 'trj_rgb', trj_rgb)
np.save(save_path + 'qtrj_rgb', qtrj_rgb)

np.save(save_path + 'trj_d', trj_d)
np.save(save_path + 'qtrj_d', qtrj_d)

np.save(save_path + 'trj_pnp', trj_pnp)
np.save(save_path + 'qtrj_pnp', qtrj_pnp)

np.save(save_path + 'trj_pnpe', trj_pnpe)
np.save(save_path + 'qtrj_pnpe', qtrj_pnpe)

np.save(save_path + 'trj_de', trj_de)
np.save(save_path + 'qtrj_de', qtrj_de)

np.save(save_path + 'trj_dp', trj_dp)
np.save(save_path + 'qtrj_dp', qtrj_dp)

np.save(save_path + 'trj_rgb', trj_rgb)
np.save(save_path + 'qtrj_rgb', qtrj_rgb)

np.save(save_path + 'trj_d', trj_d)
np.save(save_path + 'qtrj_d', qtrj_d)

np.save(save_path + 'trj_pnp', trj_pnp)
np.save(save_path + 'qtrj_pnp', qtrj_pnp)

np.save(save_path + 'trj_pnpe', trj_pnpe)
np.save(save_path + 'qtrj_pnpe', qtrj_pnpe)

np.save(save_path + 'trj_de', trj_de)
np.save(save_path + 'qtrj_de', qtrj_de)

np.save(save_path + 'trj_dp', trj_dp)
np.save(save_path + 'qtrj_dp', qtrj_dp)



np.save(save_path + 'rtrj_rgb', rtrj_rgb)
np.save(save_path + 'rqtrj_rgb', rqtrj_rgb)

np.save(save_path + 'rtrj_d', rtrj_d)
np.save(save_path + 'rqtrj_d', rqtrj_d)

np.save(save_path + 'rtrj_pnp', rtrj_pnp)
np.save(save_path + 'rqtrj_pnp', rqtrj_pnp)

np.save(save_path + 'rtrj_pnpe', rtrj_pnpe)
np.save(save_path + 'rqtrj_pnpe', rqtrj_pnpe)

np.save(save_path + 'rtrj_de', rtrj_de)
np.save(save_path + 'rqtrj_de', rqtrj_de)

np.save(save_path + 'rtrj_dp', rtrj_dp)
np.save(save_path + 'rqtrj_dp', rqtrj_dp)

np.save(save_path + 'rtrj_rgb', rtrj_rgb)
np.save(save_path + 'rqtrj_rgb', rqtrj_rgb)

np.save(save_path + 'rtrj_d', rtrj_d)
np.save(save_path + 'rqtrj_d', rqtrj_d)

np.save(save_path + 'rtrj_pnp', rtrj_pnp)
np.save(save_path + 'rqtrj_pnp', rqtrj_pnp)

np.save(save_path + 'rtrj_pnpe', rtrj_pnpe)
np.save(save_path + 'rqtrj_pnpe', rqtrj_pnpe)

np.save(save_path + 'rtrj_de', rtrj_de)
np.save(save_path + 'rqtrj_de', rqtrj_de)

np.save(save_path + 'rtrj_dp', rtrj_dp)
np.save(save_path + 'rqtrj_dp', rqtrj_dp)


# print(good_i)
good_i = np.hstack(good_i)
# print(good_i)
np.save(save_path + 'good_i', good_i)


np.save(save_path + 't', np.array([tm_rgb, tm_d, tm_de, tm_dp, tm_pnp, tm_pnpe, tm_fd, tm_ff]))


