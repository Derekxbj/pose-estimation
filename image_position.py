# from __future__ import print_function
import numpy as np
import cv2
from pandas import *
from scipy.spatial import ConvexHull

def image_position(image):
	# image = cv2.imread('image.jpg')
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
			print(cx, cy)
			if i == 0:
				blobsWhite = np.concatenate((blobsWhite, np.array([cx,cy])))
			else:
				blobsWhite = np.vstack((blobsWhite,np.array([cx,cy])))
			
		except ZeroDivisionError as detail:
			print ('Handling error:', detail)

	# print(blobsWhite)

	# I find that the openCV findContours function find all the 
	# regionprops including black and white

	# The fllowing code may helpful in the future.
	# (T, threshInv) = cv2.threshold(image, 90, 255, cv2.THRESH_BINARY_INV)
	# # cv2.imshow("Threshold Binary Inverse", threshInv)
	# opening_black = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
	# # cv2.imshow("Imopen Black", opening_black)

	# img, contours, hierarchy = cv2.findContours(opening_black, 
	# 					cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	# print(len(contours))
	# blobsBlack = np.array([])
	# for i in range(0, len(contours)):
	# 	try:
	# 		cnt = contours[i]
	# 		M = cv2.moments(cnt)
	# 		# print(M)
	# 		cx = int(M['m10']/M['m00'])
	# 		cy = int(M['m01']/M['m00'])
	# 		# print(cx, cy)
	# 		if i == 0:
	# 			blobsBlack = np.concatenate((blobsBlack, np.array([cx,cy])))
	# 		else:
	# 			blobsBlack = np.vstack((blobsBlack, np.array([cx,cy])))
			
	# 	except ZeroDivisionError as detail:
	# 		print ('Handling error:', detail)

	# print(blobsBlack)

	position = np.array([0.,0.])
	# if want to get the position with decimals, run following code
	x = blobsWhite
	x = x.astype(int)
	index = DataFrame(x).duplicated().values

	# else, run that:
	# index = DataFrame(blobsWhite).duplicated().values

	# searching the repeated elements, if element repeating, it will
	# be one of the points' position
	for i in range(index.shape[0]):
		if index[i] == True:
			position = np.vstack((position, blobsWhite[i]))
			# print(blobsWhite[i])


	position = np.delete(position, 0, 0)

	# Using convexHull method to reorder the positions
	hull = ConvexHull(position)
	temp = position.copy()
	for i in range(0,4):
		position[i-1] = temp[hull.vertices[i]]

	# The fllowing code may helpful in the future.
	# print("The following is the position of black bolobs in the image:")
	# index = DataFrame(blobsBlack).duplicated().values
	# for i in range(index.shape[0]):
	# 	if index[i] == True:
	# 		print(blobsBlack[i])

	return position