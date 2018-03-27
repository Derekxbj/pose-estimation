import cv2
import numpy as np
from math import sin,cos
from numpy import linalg as LA
from scipy.spatial import ConvexHull

class utils:
    
    @staticmethod
    def fProject(x, P_M, K):
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
        
        if ph.shape == (2,4):
            p = ph.reshape((8,1), order='F')
        elif ph.shape == (2,1):
            p = ph.reshape((2,1))

        
        # p = p.round(decimals = 100)

        return p

    @staticmethod
    def leastSquareFit(x, P_M, K, y0):

        J = np.zeros(shape = (8,6))

        for i in range(1,10):

            y = utils.fProject(x, P_M, K)

            e = 0.00001
            e1 = x + np.array([[e],[0],[0],[0],[0],[0]])
            e2 = x + np.array([[0],[e],[0],[0],[0],[0]])
            e3 = x + np.array([[0],[0],[e],[0],[0],[0]])
            e4 = x + np.array([[0],[0],[0],[e],[0],[0]])
            e5 = x + np.array([[0],[0],[0],[0],[e],[0]])
            e6 = x + np.array([[0],[0],[0],[0],[0],[e]])

            J[:,[0]] = ( utils.fProject(e1, P_M, K) - y )/e;
            J[:,[1]] = ( utils.fProject(e2, P_M, K) - y )/e;
            J[:,[2]] = ( utils.fProject(e3, P_M, K) - y )/e;
            J[:,[3]] = ( utils.fProject(e4, P_M, K) - y )/e;
            J[:,[4]] = ( utils.fProject(e5, P_M, K) - y )/e;
            J[:,[5]] = ( utils.fProject(e6, P_M, K) - y )/e;

            dy = y0 - y

            pinvJ = np.linalg.pinv(J)

            dx = pinvJ.dot(dy)

            if abs( LA.norm(dx)/LA.norm(x)) < 1e-6:
                break

            x = x + dx

        return dy,x

    @staticmethod
    def image_position(image):
        # image = cv2.imread('image2.jpg')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("Image", image)

        kernel = np.ones((5,5), np.uint8)

        (T, thresh) = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY)
        # cv2.imshow("Threshold Binary", thresh)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        # cv2.imshow("Imopen White", opening)

        img, contours, hierarchy = cv2.findContours(opening, 
                            cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # print(len(contours))
        blobsWhite = np.array([])
        for i in range(0, len(contours)):
            try:
                cnt = contours[i]
                M = cv2.moments(cnt)
                # print(M)
                # cx = int(M['m10']/M['m00'])
                # cy = int(M['m01']/M['m00'])
                cx = M['m10']/M['m00']
                cy = M['m01']/M['m00']
                # print(cx, cy)
                if i == 0:
                    blobsWhite = np.concatenate((blobsWhite, np.array([cx,cy])))
                else:
                    blobsWhite = np.vstack((blobsWhite,np.array([cx,cy])))

            except ZeroDivisionError as detail:
                print ('Handling error:', detail)

        position = np.array([0.,0.])
        x = blobsWhite

        listodd = x[::2]
        listeven = x[1::2]

        thresh = 1

        for i in range(len(listodd)):
            for j in range (len(listeven)):
                if LA.norm(listodd[i] - listeven[j]) < thresh:
                    position = np.vstack((position, listodd[i]))


        position = np.delete(position, 0, 0)
        # print(position)

        # Using convexHull method to reorder the positions
        hull = ConvexHull(position)
        temp = position.copy()
        for i in range(0,4):
            position[i-1] = temp[hull.vertices[i]]
        # print(position)


        return position