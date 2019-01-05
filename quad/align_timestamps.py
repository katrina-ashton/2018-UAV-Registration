import os
import numpy as np
import math


def Rxo(a):
	return np.array([[1, 0,           0            ],
                    [0,  math.cos(a), -math.sin(a) ],
                    [0,  math.sin(a), math.cos(a)  ]
                    ])

def Ryo(a):
	return np.array([[math.cos(a), 0, math.sin(a)  ],
                    [0,            1, 0            ],
                    [-math.sin(a), 0, math.cos(a)  ]
                    ])

def Rzo(a):
	return np.array([[math.cos(a), -math.sin(a),  0],
                    [math.sin(a),  math.cos(a),   0],
                    [0,            0,         1    ]
                    ])

def Rx(a):
	return np.array([[1,  0,            0          ],
                    [0,   math.cos(a),  math.sin(a)],
                    [0,   -math.sin(a), math.cos(a)]
                    ])

def Ry(a):
	return np.array([[math.cos(a), 0, -math.sin(a) ],
                    [0,            1, 0            ],
                    [math.sin(a),  0, math.cos(a)  ]
                    ])

def Rz(a):
	return np.array([[math.cos(a), math.sin(a),   0],
                    [-math.sin(a), math.cos(a),   0],
                    [0,            0,             1]
                    ])

def eulerAnglesToRotationMatrix(theta) :
     
    R_x = Rxo(theta[0])
                     
    R_y = Ryo(theta[1])
                 
    R_z = Rzo(theta[2])
                     
                     
    R = np.dot(R_x, np.dot( R_y, R_z ))

 
    return R.transpose()

def sample_rgb(rgb_ims, step=40):
	#circle 1: 700, 2800
	#circle 2: 745, 3023
	#rectangle: 752, 5244
	#lawnmower: 825, 6629
	start = 700 
	end = 2800 
	return rgb_ims[start:end:step]


def get_names(step=40,data_path='../../data/quad3/'):
	# data_path = '../../data/quad3/'
	save_path = './basic-reg-saves/'
	rgb_path = data_path + 'rgb/'
	depth_path = data_path + 'depth/'
	rgb_ims = os.listdir(rgb_path)
	depth_ims = os.listdir(depth_path)

	# rgb_ims = rgb_ims[118:404:2]
	rgb_ims = sample_rgb(rgb_ims,step)

	rgb_ts = []
	for n in rgb_ims:
		rgb_ts += [float(n[:-4])]

	depth_ts = []
	for n in depth_ims:
		depth_ts += [float(n[:-4])]

	rgb_ts = np.array(rgb_ts)
	depth_ts = np.array(depth_ts)

	rgb_ts_a = []
	depth_ts_a = []

	for n in rgb_ts:
		rgb_ts_a += ['{}'.format(n)+'.png']

		diff_depth = np.abs(depth_ts - n)
		i = np.argmin(diff_depth)

		depth_ts_a += ['{}'.format(depth_ts[i])+'.png']

	return rgb_ts_a, depth_ts_a

def with_gt(ts_gt, ts, data):
	data_a = []
	for t in ts_gt:
		diff = np.abs(ts - t)
		i = np.argmin(diff)
		data_a += [data[i]]

	data_a = np.vstack(data_a)

	return data_a


def gt_with_rgb(ts_gt, ts, data):
	data_a = []
	for t in ts:
		diff = np.abs(ts_gt - t)
		i = np.argmin(diff)
		data_a += [data[i]]

	data_a = np.vstack(data_a)

	return data_a

def gt_with_rgb_start(ts_gt, ts, data, start_rgb, start_gt):
	ts_gt = ts_gt - start_gt + start_rgb
	data_a = []
	for t in ts:
		diff = np.abs(ts_gt - t)
		i = np.argmin(diff)
		data_a += [data[i]]

	data_a = np.vstack(data_a)

	return data_a