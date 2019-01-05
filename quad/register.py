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

def register(method, step_size, data_path, save_root, rgb_scale):
	save_path = save_root + '/' + str(step_size) + '/'
	rgb_path = data_path + 'rgb/'
	depth_path = data_path + 'depth/'

	rgb_ims, depth_ims = align_timestamps.get_names(step_size, data_path)

	fo  = open(data_path + 'groundtruth.txt', 'r')
	fo.readline()
	trj_gt = []
	qtrj_gt = []
	t_gt = []
	for line in fo:
		ele = line[:-1].split(' ')
		if len(ele)==14:
			rotm = align_timestamps.eulerAnglesToRotationMatrix\
				([float(ele[1]),float(ele[2]),float(ele[3])])
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

	Rgt2q = align_timestamps.Rx(math.pi)
	sp = np.dot(Rgt2q,sp.transpose()).transpose()

	R1 = align_timestamps.Rx(math.pi/2+math.pi/4)
	R2 = align_timestamps.Rz(-math.pi/2) 
	Rc2q = np.dot(R2,R1)


	rot = so.rotation_matrix

	rot0 = rot.transpose()

	so = Quaternion(matrix=rot)

	pos = sp
	trj = [pos]
	qtrj = [so]
	rtrj = [pos]
	rqtrj = [so]


	mat = np.eye(3)

	fx = 616.9660034179688
	fy = 616.8399047851562
	cx = 328.9248962402344 
	cy = 230.74755859375
	focal = np.sqrt(fx**2+fy**2)
	pp = (cx,cy)

	p = 0.9999

	cmat = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]])

	tm_m = 0
	tm_fd = 0
	tm_ff = 0


	for i in range(len(rgb_ims)-1):
		img1 = cv2.imread(rgb_path + rgb_ims[i],0)   # queryImage
		img2 = cv2.imread(rgb_path + rgb_ims[i+1],0) # trainImage

		mask = np.array(img1.shape, dtype=np.uint8)


		tm = time.time()
		# Initiate SIFT detector
		sift = cv2.xfeatures2d.SIFT_create()

		# find the keypoints and descriptors with SIFT - image frame
		kp1, des1 = sift.detectAndCompute(img1,None)
		kp2, des2 = sift.detectAndCompute(img2,None)

		des1 = des1.astype(np.uint8)
		des2 = des2.astype(np.uint8)


		# create BFMatcher object
		bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

		# Match descriptors.
		matches = bf.match(des1,des2) #train, query ?

		# Sort them in the order of their distance.
		matches = sorted(matches, key = lambda x:x.distance)

		#Get points
		pt1 = []
		pt2 = []

		for j in range(len(matches)-1):
			pt1 += [kp1[matches[j].queryIdx].pt]
			pt2 += [kp2[matches[j].trainIdx].pt]

		pt1 = np.vstack(pt1)
		pt2 = np.vstack(pt2)

		tm_ff += time.time() - tm

		imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches, None)
		plt.imshow(imgm)
		plt.savefig('matches/{}_before.png'.format(rgb_ims[i][:-4]))
		plt.close()

		#Set up 3d points
		if(method=='d'):
			tm = time.time()
			d_img1 = cv2.imread(depth_path + depth_ims[i],0)   # queryImage
			d_img2 = cv2.imread(depth_path + depth_ims[i+1],0) # trainImage

			pt1 = pt1.astype(np.uint32)
			pt2 = pt2.astype(np.uint32)

			pt2f = []

			P = []
			Q = []

			rgb_img1 = cv2.imread(rgb_path + rgb_ims[i])   # queryImage
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

					matches3 += [matches[j]]

			P = np.vstack(P)
			Q = np.vstack(Q)
			Cp = np.vstack(Cp)/255.0
			Cq = np.vstack(Cq)/255.0

			tm_fd += time.time() - tm

		elif(method=='dp'):
			tm = time.time()
			d_img1 = cv2.imread(depth_path + depth_ims[i],0)   # queryImage
			d_img2 = cv2.imread(depth_path + depth_ims[i+1],0) # trainImage

			pt1 = pt1.astype(np.uint32)
			pt2 = pt2.astype(np.uint32)

			pt2f = []

			P = []
			Q = []

			rgb_img1 = cv2.imread(rgb_path + rgb_ims[i])   # queryImage
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

					pt2f += [[float(pt2[j,0]), float(pt2[j,1])]]

					Cp += [rgb_img1[pt1[j,1], pt1[j,0],:]]
					Cq += [rgb_img2[pt1[j,1], pt1[j,0],:]]

					matches3 += [matches[j]]

			P = np.vstack(P)
			Q = np.vstack(Q)
			Cp = np.vstack(Cp)/255.0
			Cq = np.vstack(Cq)/255.0

			pt2f = np.vstack(pt2f)

			tm_fd += time.time() - tm			

		elif(method=='pnp' or method=='pnpe'):
			tm = time.time()
			d_img1 = cv2.imread(depth_path + depth_ims[i],0)   # queryImage
			d_img2 = cv2.imread(depth_path + depth_ims[i+1],0) # trainImage

			pt1 = pt1.astype(np.uint32)
			pt2 = pt2.astype(np.uint32)

			pt2f = []

			P = []

			matches3 = []

			for j in range(pt1.shape[0]):
				#convert to camera frame 1 first
				d1 = d_img1[pt1[j,1], pt1[j,0]]
				d2 = d_img2[pt2[j,1], pt2[j,0]]

				if d1 > 0 and d2 >0:
					P += [coord_transforms.im_to_cam(pt1[j,0], pt1[j,1], d1)]
					pt2f += [[float(pt2[j,0]), float(pt2[j,1])]]

					matches3 += [matches[j]]

			pt2f = np.vstack(pt2f)
			P = np.vstack(P)

			tm_fd += time.time() - tm			

		#Do registration
		if(method=='rgb'):
			tm = time.time()
			E = cv2.findEssentialMat(pt2, pt1, focal, pp, cv2.RANSAC, p, 1.0, None)
			R = np.zeros((3,3))
			t = np.zeros((3,1))
			cv2.recoverPose(E[0], pt2, pt1, R, t, focal, pp, None) #camera frame 1??

			tm_m += time.time() - tm

			matches_good = []
			for m in range(len(E[1])):
				if E[1][m] == 1:
					matches_good += [matches[m]]

			imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
			plt.imshow(imgm)
			plt.savefig('matches/{}_essential.png'.format(rgb_ims[i][:-4]))
			plt.close()

			R = np.transpose(R)
			t = -t/rgb_scale

		elif(method=='d'):
			tm = time.time()
			R, t, inliers = kabsch.kabsch(P, Q, p=p, v=0.5, dist_thresh = 0.1)
			tm_m += time.time() - tm
			
			matches_good = []
			if not (inliers is None): 
				for m in inliers:
					matches_good += [matches3[m]]

				imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
				plt.imshow(imgm)
				plt.savefig('matches/{}_kabsch.png'.format(rgb_ims[i][:-4]))
				plt.close()

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
			plt.close()

			Pa = np.dot(P, R.transpose()) + np.repeat(t.transpose(), P.shape[0], axis=0)

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
			plt.close()

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
			plt.close()

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
			plt.close()

		elif(method=='de'):
			E = cv2.findEssentialMat(pt2, pt1, focal, pp, cv2.RANSAC, p, 1.0, None)

			if len(E[1])<5:
				R = np.eye(3)
				t = np.array([0,0,0])
			else:

				tm = time.time()
				d_img1 = cv2.imread(depth_path + depth_ims[i],0)   # queryImage
				d_img2 = cv2.imread(depth_path + depth_ims[i+1],0) # trainImage

				pt1 = pt1.astype(np.uint32)
				pt2 = pt2.astype(np.uint32)

				Pe = []
				Qe = []

				for j in range(pt1.shape[0]):
					#convert to camera frame 1 first
					d1 = d_img1[pt1[j,1], pt1[j,0]]
					d2 = d_img2[pt2[j,1], pt2[j,0]]

					if d1 > 0 and d2 >0:
						if E[1][j]==1:
							Pe += [coord_transforms.im_to_cam(pt1[j,0], pt1[j,1], d1)]
							Qe += [coord_transforms.im_to_cam(pt2[j,0], pt2[j,1], d2)]

				Pe = np.vstack(Pe)
				Qe = np.vstack(Qe)

				tm_fd += time.time() - tm

				tm = time.time()
				R, t= kabsch.find_transform(Pe, Qe)
				tm_m += time.time() - tm

		elif(method=='dp'):
			v = 0.8
			m = 5
			N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))
			dist_thresh = 1 #0.05

			P = P.reshape((P.shape[0],1,3))
			pt2f = pt2f.reshape((pt2f.shape[0],1,2))

			_, R_vec, t, in_pnp = cv2.solvePnPRansac(P, pt2f, cmat, None, \
				iterationsCount=N, reprojectionError=dist_thresh, confidence=p, \
				flags=cv2.SOLVEPNP_ITERATIVE)
			if in_pnp is None:
				R = np.eye(3)
				t = np.array([0,0,0])
			else:
				Pi = P[in_pnp[:],:]
				Qi = Q[in_pnp[:],:]
				Pi = Pi.reshape((Pi.shape[0],3))
				Qi = Qi.reshape((Qi.shape[0],3))
				# print(i)
				tm = time.time()
				R, t = kabsch.find_transform(Pi, Qi)
				tm_m += time.time() - tm

		elif(method=='pnp'):
			v = 0.8
			m = 5
			N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))
			dist_thresh = 1 #0.05

			P = P.reshape((P.shape[0],1,3))
			pt2f = pt2f.reshape((pt2f.shape[0],1,2))

			tm = time.time()
			_, R_vec, t, in_pnp = cv2.solvePnPRansac(P, pt2f, cmat, None, \
				iterationsCount=N, reprojectionError=dist_thresh, confidence=p, \
				flags=cv2.SOLVEPNP_ITERATIVE)
			tm_m += time.time() - tm

			R = np.zeros((3,3))
			cv2.Rodrigues(R_vec,R)

			matches_good = []
			if in_pnp is not None:
				for m in in_pnp:
					matches_good += [matches3[m[0]]]

				imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
				plt.imshow(imgm)
				plt.savefig('matches/{}_pnp.png'.format(rgb_ims[i][:-4]))
				plt.close()

		elif(method=='pnpe'):
			v = 0.8
			m = 5
			N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))
			dist_thresh = 1 #0.05

			P = P.reshape((P.shape[0],1,3))
			pt2f = pt2f.reshape((pt2f.shape[0],1,2))

			tm = time.time()
			_, R_vec, t, in_pnp = cv2.solvePnPRansac(P, pt2f, cmat, None, iterationsCount=N, \
				reprojectionError=dist_thresh, confidence=p, flags=cv2.SOLVEPNP_EPNP)
			tm_m += time.time() - tm

			R = np.zeros((3,3))
			cv2.Rodrigues(R_vec,R)

			matches_good = []
			if in_pnp is not None:
				for m in in_pnp:
					matches_good += [matches3[m[0]]]

				imgm = cv2.drawMatches(img1, kp1, img2, kp2, matches_good, None)
				plt.imshow(imgm)
				plt.savefig('matches/{}_pnpe.png'.format(rgb_ims[i][:-4]))
				plt.close()

		#Add to trajectory

		Rq = qtrj_gt[i][0].rotation_matrix.transpose()
		pq = np.dot(Rgt2q,trj_gt[i,:].transpose()).transpose()

		R = np.dot(np.dot(Rc2q,R),Rc2q.transpose())
		t = np.dot(Rc2q,t)

		q_c = np.dot(rot, R)
		pos = pos + np.transpose(np.dot(rot.transpose(),t))
		rot = q_c

		if (len(pos)!=3):
			pos = pos[0]
		trj += [pos]
		qtrj += [Quaternion(matrix=q_c.transpose())]


		t = np.dot(Rc2q.transpose(),t)
		
		qr = np.dot(Rq, R)
		rpos = pq + np.transpose(np.dot(Rq.transpose(),t))

		if (len(rpos)!=3):
			rpos = rpos[0]
		rtrj += [rpos]
		rqtrj += [Quaternion(matrix=qr.transpose())]


	#Save
	trj = np.vstack(trj)
	qtrj = np.vstack(qtrj)
	rtrj = np.vstack(rtrj)
	rqtrj = np.vstack(rqtrj)

	np.save(save_path + 'trj_' + method, trj)
	np.save(save_path + 'qtrj_' + method, qtrj)
	np.save(save_path + 'rtrj_' + method, rtrj)
	np.save(save_path + 'rqtrj_' + method, rqtrj)

	np.save(save_path + 't' + method, np.array([tm_ff, tm_fd, tm_m]))