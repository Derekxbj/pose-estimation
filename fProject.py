import numpy as np 
from math import sin,cos
def fProject(x, P_M, K):
	# x = np.array([0,0,0,0,0,23])

	# # # The point in the model's coordinate system(cm)
	# P_M = np.array([[-5,5,5,-5],
	#                [-5,-5,5,5],
	#                [0,0,0,0],
	#                [1,1,1,1]])


	# # # Define camera parameters
	# f = 504
	# cx = 320
	# cy = 240

	# # # intrinsic parameter matrix
	# K = np.array([[f,0,cx],
	#               [0,f,cy],
	#               [0,0,1]])

	# Get pose parametes
	ax = x[0]
	ay = x[1]
	az = x[2]
	tx = x[3]
	ty = x[4]
	tz = x[5]

	# Rotation matrix using Euler angle
	Rx = np.array([[1,0,0],
				  [0,cos(ax),-sin(ax)],
				  [0,sin(ax),cos(ax)]])

	Ry = np.array([[cos(ay),0,sin(ay)],
				   [0,1,0],
				   [-sin(ay),0,cos(ay)]])

	Rz = np.array([[cos(az),-sin(az),0],
				   [sin(az),cos(az),0],
				   [0,0,1]])
	R = (Rz.dot(Ry)).dot(Rx)


	# Extrinsic camera matrix (rotation and translation)
	T = np.array([tx , ty, tz])
	Mext = np.hstack((R,T))


	# Projects points
	ph = (K.dot(Mext)).dot(P_M)
	ph[0,:] = ph[0,:]/ph[2,:]
	ph[1,:] = ph[1,:]/ph[2,:]
	ph = ph[:2,:]

	p = ph.reshape((8,1), order='F')
	# p = p.round(decimals = 100)

	return p




