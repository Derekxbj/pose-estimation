import numpy as np
import cv2
from fProject import *
from image_position import *
from LeastSquareFit import *
import time

start_time = time.time()

image = cv2.imread('image10.jpg')


# The point in the model's coordinate system(cm)
P_M = np.array([[-5,5,5,-5],
                [-5,-5,5,5],
                [0,0,0,0],
                [1,1,1,1]])

# Define camera parameters
f = 504
cx = 320
cy = 240

# intrinsic parameter matrix
K = np.array([[f,0,cx],
              [0,f,cy],
              [0,0,1]])

# Make an initial guess of the pose 
x0 = np.array([[0],[0],[0],[0],[0],[23]])

# Find the position of points on the image
position = image_position(image)

y = np.zeros(shape = (8,1))

y[0] = position[0,0]
y[1] = position[0,1] 
y[2] = position[1,0] 
y[3] = position[1,1] 
y[4] = position[2,0] 
y[5] = position[2,1] 
y[6] = position[3,0] 
y[7] = position[3,1] 

x = leastSquareFit(x0, P_M, K, y)

x[0] = np.rad2deg(x[0])
x[1] = np.rad2deg(x[1])
x[2] = np.rad2deg(x[2])

print(x)

print("--- %s seconds ---" % (time.time() - start_time))