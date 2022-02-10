import os
import cv2
import numpy as np

class filtrage:
	@staticmethod
	def preprocess(image_raw,fast_algo):
		
		image_filtered = cv2.cvtColor(image_raw, cv2.COLOR_BGR2GRAY)
		image_filtered = cv2.medianBlur(image_filtered, 25)
		image_filtered = cv2.bitwise_not(image_filtered)

		p1 = 1501
		if(fast_algo):
			p1 = 501
		image_filtered = cv2.adaptiveThreshold(image_filtered,255,cv2.ADAPTIVE_THRESH_MEAN_C,cv2.THRESH_BINARY,p1,10)#1501

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
			(x,y,w,h) = cv2.boundingRect(closest_contour)
			centerM = np.array( [ int(x+(w/2)), int(y+(h/2)) ] )
			closest_dist = np.linalg.norm(centerImg-centerM)
			for c in contours:
				(x,y,w,h) = cv2.boundingRect(c)
				centerM = np.array( [ int(x+(w/2)), int(y+(h/2)) ] )
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
		if(fast_algo):
			border=20
		image_filtered = image_filtered[y-border:y+h+border,x-border:x+w+border]
		offset = (x-border,y-border)

		return cv2.bitwise_not(image_filtered),offset,False

