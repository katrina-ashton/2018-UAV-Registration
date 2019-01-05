from eval import get_trjs, get_rtrjs, get_rotm, save_plot, compare, process_comp_method, process_comp_all, process_comp_best
from pyquaternion import Quaternion
import numpy as np

data_path = '../../data/quad3/'
save_path = './basic-reg-saves/20/'
step =20

trj_gt, qtrj_gt, trj_rgb, qtrj_rgb, trj_d, qtrj_d, trj_pnp, qtrj_pnp, trj_de, qtrj_de, trj_dp, qtrj_dp, trj_pnpe, qtrj_pnpe, rgb_ts = get_trjs(data_path, save_path, step)

total_tr = 0
total_a = 0

for i in range(len(qtrj_gt)-1):
	total_tr += np.sqrt(np.sum(np.square(trj_gt[i]-trj_gt[i+1])))
	r = qtrj_gt[i,0].inverse * qtrj_gt[i+1,0]
	# print(abs(r.radians))
	total_a += abs(r.radians)

print(len(trj_gt))
print(len(trj_rgb))

print(rgb_ts[-1]-rgb_ts[0])
print(total_tr)
print(total_a)

#(step 1)
#circle 1:
# [69.88999987]
# 11.87431077970471
# 11.928861468329714

#circle 2:
# [75.93000007]
# 13.404915561036152
# 12.419477223200264

#rectangle:
# [150.51999998]
# 14.972810856470433
# 17.207008968439762

#lawnmower:
# [194.96000004]
# 20.51718719703204
# 38.382665866625544


