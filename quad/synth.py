import numpy as np
import math
from pyquaternion import Quaternion
import kabsch
from eval import compare

R = Quaternion(axis=[0.5,0.2,0.3],angle=math.pi/5).rotation_matrix
t = np.array([1,0.4,10**-8]).reshape(3,1)

noise = 1e-1
n = 100
P = np.random.rand(3,n)
Q = np.dot(R,P) + np.repeat(t,P.shape[1],axis=1) 

nv = noise*np.random.rand(3,n)
for i in range(int(n*0.5)):
	nv[:,np.random.randint(n)] = np.array([0,0,0]).transpose()

Q += nv

P = P.transpose()
Q = Q.transpose()

Rk, tk, _ = kabsch.kabsch(P,Q, 0.9999, v=0.5, dist_thresh = noise)

# Rk, tk = kabsch.find_transform(P,Q)

print(R)
print(Rk)
print(t)
print(tk)

et, eR = compare(R,t,Rk,tk)

print(et)
print(eR)