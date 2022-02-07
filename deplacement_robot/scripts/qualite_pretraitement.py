import os
import cv2
import numpy as np

class filtrage:
    @staticmethod
    def preprocess(image_raw):
	

	#crop_size = 100
	#crop_img = image_raw[crop_size:-crop_size, crop_size:-crop_size]


        image_filtered = cv2.cvtColor(image_raw, cv2.COLOR_BGR2GRAY)
        image_filtered = cv2.medianBlur(image_filtered, 25)
        image_filtered = cv2.bitwise_not(image_filtered)

        image_filtered = cv2.adaptiveThreshold(image_filtered,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,1501,10)#1501

        des = cv2.bitwise_not(image_filtered)
        contours,_ = cv2.findContours(des,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)[-2:]

	if(not contours):
		return None,None,True
		

	if(len(contours) != 1) :
		print("multiple shapes detected, croping")
		print("detected ",len(contours) )
		#finding centered contour
		imgW,imgH = image_filtered.shape
		centerImg = np.array([imgW/2,imgH/2])

		closest_contour = contours[0]
		M = cv2.moments(closest_contour)
		centerM = np.array([int(M['m10']/M['m00']),int(M['m01']/M['m00'])])
		closest_dist = np.linalg.norm(centerImg-centerM)
		for c in contours:
			M = cv2.moments(c)
			centerM = np.array([int(M['m10']/M['m00']),int(M['m01']/M['m00'])])
			d = np.linalg.norm(centerImg-centerM)
			if(d<closest_dist):
				closest_dist = d
				closest_contour = c

		contour = closest_contour

	else:

		contour = contours[0]

	#croping around contour to save process time:
	(x,y,w,h) = cv2.boundingRect(contour)
	border = 100
	image_filtered = image_filtered[y-border:y+h+border,x-border:x+w+border]
	offset = (x-border,y-border)


	print("shape filtered = ",image_filtered.shape)

        return cv2.bitwise_not(image_filtered),offset,False

