import numpy as np
import cv2

def kabsch(P,Q,p,v):
	m = 3
	N = np.int(np.ceil(np.log(1-p)/np.log(1-(1-v)**m)))

	frac_thresh = 0.05
	dist_thresh = 1

	num_points = P.shape[0]

	best_dist = 1e10
	mat = np.eye(3)

	for n in range(N):
		i = np.random.randint(0, num_points, size=m)

		Pn = P[i,:]
		Qn = Q[i,:]


		p0 = (np.sum(Pn, axis=0)/Pn.shape[0]).reshape(1,3)
		q0 = (np.sum(Qn, axis=0)/Qn.shape[0]).reshape(1,3)

		t_d = (q0-p0).reshape(3,1)


		P1 = Pn - np.repeat(p0,Pn.shape[0],axis=0).astype(np.float32)
		Q1 = Qn - np.repeat(q0,Qn.shape[0],axis=0).astype(np.float32)

		H = np.dot(np.transpose(P1),Q1)

		S, U, Vt = cv2.SVDecomp(H)

		V = Vt.transpose()
		Ut = U.transpose()

		d = np.linalg.det(np.dot(V,Ut))

		mat[2,2] = d

		R_d = np.dot(np.dot(V,mat),Ut)

		Pa = np.dot(P, R_d.transpose()) + np.repeat(t_d.transpose(), num_points, axis=0)

		# Pa = np.zeros(P.shape)

		# for j in range(P.shape[0]):
		# 	Pa[j,:] = np.dot(R_d, P[j,:].transpose()) + t_d.transpose()

		dist = np.sqrt(np.sum(np.square(Pa - Q), axis=1))

		good_i = np.where(dist<dist_thresh)[0]

		# print(dist)

		# print(len(good_i))

		if (len(good_i))/num_points > frac_thresh:
			# print(len(good_i))
			Pn = P[good_i,:]
			Qn = Q[good_i,:]

			p0 = (np.sum(Pn, axis=0)/Pn.shape[0]).reshape(1,3)
			q0 = (np.sum(Qn, axis=0)/Qn.shape[0]).reshape(1,3)

			t_d = (q0-p0).reshape(3,1)


			P1 = Pn - np.repeat(p0,Pn.shape[0],axis=0).astype(np.float32)
			Q1 = Qn - np.repeat(q0,Qn.shape[0],axis=0).astype(np.float32)

			H = np.dot(np.transpose(P1),Q1)

			S, U, Vt = cv2.SVDecomp(H)

			V = Vt.transpose()
			Ut = U.transpose()

			d = np.linalg.det(np.dot(V,Ut))

			mat[2,2] = d

			R_d = np.dot(np.dot(V,mat),Ut)

			Pa = np.dot(Pn, R_d.transpose()) + np.repeat(t_d.transpose(), len(good_i), axis=0)

			# Pa = np.zeros(Pn.shape)

			# for j in range(Pn.shape[0]):
			# 	Pa[j,:] = np.dot(R_d, Pn[j,:].transpose()) + t_d.transpose()

			dist = np.mean(np.sqrt(np.sum(np.square(Pa - Qn), axis=1)))

			if dist < best_dist:
				best_good_i = good_i
				best_dist = dist
				best_R_d = R_d
				best_t_d = t_d

	return best_R_d, best_t_d, best_good_i