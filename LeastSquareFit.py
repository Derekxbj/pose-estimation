from fProject import *
import numpy as np


def leastSquareFit(x, P_M, K, y0):

	J = np.zeros(shape = (8,6))

	for i in range(1,10):

		y = fProject(x, P_M, K)

		e = 0.00001
		e1 = x + np.array([[e],[0],[0],[0],[0],[0]])
		e2 = x + np.array([[0],[e],[0],[0],[0],[0]])
		e3 = x + np.array([[0],[0],[e],[0],[0],[0]])
		e4 = x + np.array([[0],[0],[0],[e],[0],[0]])
		e5 = x + np.array([[0],[0],[0],[0],[e],[0]])
		e6 = x + np.array([[0],[0],[0],[0],[0],[e]])

		J[:,[0]] = ( fProject(e1, P_M, K) - y )/e;
		J[:,[1]] = ( fProject(e2, P_M, K) - y )/e;
		J[:,[2]] = ( fProject(e3, P_M, K) - y )/e;
		J[:,[3]] = ( fProject(e4, P_M, K) - y )/e;
		J[:,[4]] = ( fProject(e5, P_M, K) - y )/e;
		J[:,[5]] = ( fProject(e6, P_M, K) - y )/e;

		dy = y0 - y

		pinvJ = np.linalg.pinv(J)

		dx = pinvJ.dot(dy)

		x = x + dx

	return x