import numpy as np

import register
from eval import get_trjs, get_rtrjs, get_rotm, save_plot, compare, \
	process_comp_method, process_comp_all, process_comp_best

rgb_scales = [7,4.5,4,3,2,1,1,1]
stepsizes = [20,30,40,50,60,70,80,90]
methods = ['rgb','d','de','dp','pnp','pnpe']

data_path = '../../data/new-lawnmower/'
save_root = './basic-reg-saves-new-lawnmower/'

for i in range(len(stepsizes)):
	step = stepsizes[i]
	rgb_scale = rgb_scales[i]
	print(step)
	for method in methods:  
		print(method)
		register.register(method, step, data_path, save_root, rgb_scale)


aeR_rgb = []
aet_rgb = []
aeR_d = []
aet_d = []
aeR_de = []
aet_de = []
aeR_dp = []
aet_dp = []
aeR_pnp = []
aet_pnp = []
aeR_pnpe = []
aet_pnpe = []

reR_rgb = []
ret_rgb = []
reR_d = []
ret_d = []
reR_de = []
ret_de = []
reR_dp = []
ret_dp = []
reR_pnp = []
ret_pnp = []
reR_pnpe = []
ret_pnpe = []

tsa = []

for step in stepsizes:
	save_path = save_root + str(step) + '/'

	trj_gt, qtrj_gt, trj_rgb, qtrj_rgb, trj_d, qtrj_d, trj_pnp, qtrj_pnp, \
		trj_de, qtrj_de, j_dp, qtrj_dp, trj_pnpe, qtrj_pnpe, rgb_ts \
		= get_trjs(data_path, save_path, step)
	rot_gt, rot_rgb, rot_d, rot_pnp, rot_de, rot_dp, rot_pnpe \
		= get_rotm(qtrj_gt, qtrj_rgb, qtrj_d, qtrj_pnp, qtrj_de, \
		qtrj_dp, qtrj_pnpe)


	lim = 1.5
	s = 0.1
	save_plot(rot_gt,trj_gt,lim,s,'Ground truth trajectory','atrj_gt',save_path)
	save_plot(rot_rgb,trj_rgb,lim,s,'Trajectory estimated using Essential Matrix',\
		'atrj_rgb',save_path)
	save_plot(rot_d,trj_d,lim,s,'Trajectory estimated using Kabsch',\
		'atrj_d',save_path)
	save_plot(rot_de,trj_de,lim,s,'Trajectory estimated using Kabsch (EM inliers)',\
		'atrj_de',save_path)
	save_plot(rot_dp,trj_dp,lim,s,'Trajectory estimated using Kabsch (PnP inliers)',\
		'atrj_dp',save_path)
	save_plot(rot_pnp,trj_pnp,lim,s,'Trajectory estimated using iterative PnP',\
		'atrj_pnp',save_path)
	save_plot(rot_pnpe,trj_pnpe,lim,s,'Trajectory estimated using EPnP',\
		'atrj_pnpe',save_path)

	et_rgb, eR_rgb = compare(rot_gt,trj_gt,rot_rgb,trj_rgb)
	et_d, eR_d = compare(rot_gt,trj_gt,rot_d,trj_d)
	et_de, eR_de = compare(rot_gt,trj_gt,rot_de,trj_de)
	et_dp, eR_dp = compare(rot_gt,trj_gt,rot_dp,trj_dp)
	et_pnp, eR_pnp = compare(rot_gt,trj_gt,rot_pnp,trj_pnp)
	et_pnpe, eR_pnpe = compare(rot_gt,trj_gt,rot_pnpe,trj_pnpe)

	aeR_rgb += [eR_rgb]
	aet_rgb += [et_rgb]
	aeR_d += [eR_d]
	aet_d += [et_d]
	aeR_de += [eR_de]
	aet_de += [et_de]
	aeR_dp += [eR_dp]
	aet_dp += [et_dp]
	aeR_pnp += [eR_pnp]
	aet_pnp += [et_pnp]
	aeR_pnpe += [eR_pnpe]
	aet_pnpe += [et_pnpe]

	tsa += [rgb_ts]

	ets = np.vstack([et_rgb, et_d, et_de, et_dp, et_pnp, et_pnpe])
	eRs = np.vstack([eR_rgb, eR_d, eR_de, eR_dp, eR_pnp, eR_pnpe])

	process_comp_all(rgb_ts, ets, eRs, save_path, 'a')


	trj_gt, qtrj_gt, trj_rgb, qtrj_rgb, trj_d, qtrj_d, trj_pnp, qtrj_pnp, trj_de, \
		qtrj_de, trj_dp, qtrj_dp, trj_pnpe, qtrj_pnpe, rgb_ts \
		= get_rtrjs(data_path, save_path, step)
	rot_gt, rot_rgb, rot_d, rot_pnp, rot_de, rot_dp, rot_pnpe \
		= get_rotm(qtrj_gt, qtrj_rgb, qtrj_d, qtrj_pnp, qtrj_de, \
			qtrj_dp, qtrj_pnpe)

	lim = 1.5
	s = 0.1
	save_plot(rot_gt,trj_gt,lim,s,'Ground truth trajectory','rtrj_gt',save_path)
	save_plot(rot_rgb,trj_rgb,lim,s,'Trajectory estimated using Essential Matrix',\
		'rtrj_rgb',save_path)
	save_plot(rot_d,trj_d,lim,s,'Trajectory estimated using Kabsch','rtrj_d',\
		save_path)
	save_plot(rot_de,trj_de,lim,s,'Trajectory estimated using Kabsch (EM inliers)',\
		'rtrj_de',save_path)
	save_plot(rot_dp,trj_dp,lim,s,'Trajectory estimated using Kabsch (PnP inliers)',\
		'rtrj_dp',save_path)
	save_plot(rot_pnp,trj_pnp,lim,s,'Trajectory estimated using iterative PnP',\
		'rtrj_pnp',save_path)
	save_plot(rot_pnpe,trj_pnpe,lim,s,'Trajectory estimated using EPnP','rtrj_pnpe',\
		save_path)

	et_rgb, eR_rgb = compare(rot_gt,trj_gt,rot_rgb,trj_rgb)
	et_d, eR_d = compare(rot_gt,trj_gt,rot_d,trj_d)
	et_de, eR_de = compare(rot_gt,trj_gt,rot_de,trj_de)
	et_dp, eR_dp = compare(rot_gt,trj_gt,rot_dp,trj_dp)
	et_pnp, eR_pnp = compare(rot_gt,trj_gt,rot_pnp,trj_pnp)
	et_pnpe, eR_pnpe = compare(rot_gt,trj_gt,rot_pnpe,trj_pnpe)

	reR_rgb += [eR_rgb]
	ret_rgb += [et_rgb]
	reR_d += [eR_d]
	ret_d += [et_d]
	reR_de += [eR_de]
	ret_de += [et_de]
	reR_dp += [eR_dp]
	ret_dp += [et_dp]
	reR_pnp += [eR_pnp]
	ret_pnp += [et_pnp]
	reR_pnpe += [eR_pnpe]
	ret_pnpe += [et_pnpe]

	ets = np.vstack([et_rgb, et_d, et_de, et_dp, et_pnp, et_pnpe])
	eRs = np.vstack([eR_rgb, eR_d, eR_de, eR_dp, eR_pnp, eR_pnpe])

	process_comp_all(rgb_ts, ets, eRs, save_path, 'r')


save_path = save_root + 'methods/'
process_comp_method(tsa, aet_rgb, aeR_rgb, save_path, "Essential Matrix", \
	'rgb' ,pre='a')
process_comp_method(tsa, aet_d, aeR_d, save_path, "Kabsch", 'd' ,pre='a')
process_comp_method(tsa, aet_de, aeR_de, save_path, \
	"Kabsch (Essential Matrix inliers)", 'de' ,pre='a')
process_comp_method(tsa, aet_dp, aeR_dp, save_path, "Kabsch (PnP inliers)", \
	'dp' ,pre='a')
process_comp_method(tsa, aet_pnp, aeR_pnp, save_path, "PnP (iterative)", \
	'pnp' ,pre='a')
process_comp_method(tsa, aet_pnpe, aeR_pnpe, save_path, "EPnP", 'pnpe', pre='a')

print('\n')

process_comp_method(tsa, ret_rgb, aeR_rgb, save_path, "Essential Matrix", 'rgb' ,pre='r')
process_comp_method(tsa, ret_d, aeR_d, save_path, "Kabsch", 'd' ,pre='r')
process_comp_method(tsa, ret_de, aeR_de, save_path, \
	"Kabsch (Essential Matrix inliers)", 'de' ,pre='r')
process_comp_method(tsa, ret_dp, aeR_dp, save_path, "Kabsch (PnP inliers)", \
	'dp' ,pre='r')
process_comp_method(tsa, ret_pnp, aeR_pnp, save_path, "PnP (iterative)", 'pnp' ,pre='r')
process_comp_method(tsa, ret_pnpe, aeR_pnpe, save_path, "EPnP", 'pnpe' ,pre='r')

eta = [aet_rgb, aet_d, aet_de, aet_dp, aet_pnp, aet_pnpe]
eRa = [aeR_rgb, aeR_d, aeR_de, aeR_dp, aeR_pnp, aeR_pnpe]

etr = [ret_rgb, ret_d, ret_de, ret_dp, ret_pnp, ret_pnpe]
eRr = [reR_rgb, reR_d, reR_de, reR_dp, reR_pnp, reR_pnpe]

idx = [1,4,0,2,1,1]
names = ['Essential Matrix (skip 30)', 'Kabsch (skip 60)', \
	'Kabsch (EM inlers) (skip 20)', 'Kabsch (PnP inliers) (skip 40)', \
	'PnP (iterative) (skip 30)', 'EPnP (skip 30)']

process_comp_best(tsa, idx, eta, eRa, save_path, names, 'a')
process_comp_best(tsa, idx, etr, eRr, save_path, names, 'r')

suff = ['rgb','d','de','dp','pnp','pnpe']
idx = [1,4,0,2,1,1]
tf = 2100
n = []

for i in range(len(idx)):
	n += [np.floor(tf/((idx[i]+2)*10))]

print(n)
for i in range(len(idx)):
	t = np.load(save_root + str(stepsizes[idx[i]]) + '/t' + suff[i] + '.npy')
	print(t)
	print(np.sum(t))
	print(np.sum(t)/n[i])
