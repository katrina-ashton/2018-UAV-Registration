import numpy as np
import cv2

def kabsch(P,Q,p,v, dist_thresh = 0.03):
	np.random.seed(0)
	m = 5
	N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))

	frac_thresh = max(0.05, m/N)

	num_points = P.shape[0]

	best_dist = 1e10
	best_good_i = None

	for n in range(N):
		i = np.random.randint(0, num_points, size=m)

		Pn = P[i,:]
		Qn = Q[i,:]

		R_d, t_d = find_transform(Pn, Qn)

		Pa = np.dot(P, R_d.transpose()) + np.repeat(t_d.transpose(), num_points, axis=0)

		# Pa = np.zeros(P.shape)

		# for j in range(P.shape[0]):
		# 	Pa[j,:] = np.dot(R_d, P[j,:].transpose()) + t_d.transpose()

		dist = np.sqrt(np.sum(np.square(Pa - Q), axis=1))

		# print(min(dist))

		good_i = np.where(dist<dist_thresh)[0]

		# print(dist)

		# print(len(good_i))

		if (len(good_i)/N) > frac_thresh:
			Pn = P[good_i,:]
			Qn = Q[good_i,:]

			R_d, t_d = find_transform(Pn, Qn)
			Pa = np.dot(Pn, R_d.transpose()) + np.repeat(t_d.transpose(), len(good_i), axis=0)

			dist = np.sum(np.sqrt(np.sum(np.square(Pa - Qn), axis=1)))
			if dist < best_dist:
				best_dist = dist
				best_R_d = R_d
				best_t_d = t_d
				best_good_i = good_i
			# print(len(good_i))

	
	if (best_good_i is None):
		print(dist_thresh)
		if(dist_thresh>20):
			return np.eye(3), np.array([[0,0,0]]).transpose(), None
		return kabsch(P,Q,p,v, dist_thresh =dist_thresh*2)


	return best_R_d, best_t_d, best_good_i


def find_transform(Pn, Qn):
	mat = np.eye(3)

	p0 = (np.sum(Pn, axis=0)/Pn.shape[0]).reshape(1,3)
	q0 = (np.sum(Qn, axis=0)/Qn.shape[0]).reshape(1,3)

	


	P1 = Pn - np.repeat(p0,Pn.shape[0],axis=0).astype(np.float32)
	Q1 = Qn - np.repeat(q0,Qn.shape[0],axis=0).astype(np.float32)

	C = np.dot(P1.transpose(),Q1) #Nx3, not 3xN

	S, V, Wt = cv2.SVDecomp(C)

	Vt = V.transpose()
	W = Wt.transpose()

	d = np.round(np.linalg.det(np.dot(W,Vt)))

	mat[2,2] = d

	R_d = np.dot(np.dot(W,mat),Vt)

	t_d = q0.reshape(3,1)-np.dot(R_d,p0.reshape(3,1))

	return R_d, t_d
	