import os
from tkinter import image_types
import cv2
import numpy as np

class filtrage:
	
	def get_circularite(contour):
		aire = cv2.contourArea(contour)
		perimetre = cv2.arcLength(contour,True)
		return ( 4*np.pi*aire ) / (perimetre**2)
	
	
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
			print("EMPTY IMAGE (NO HOLES FOUND)")
			return None,None,True
			
		
		if(len(contours) != 1) :
			print("multiple shapes detected, croping")
			print("detected ",len(contours) )

			#finding contour that look like a cricle
			most_circular = contours[0]
			best_cirularity = filtrage.get_circularite(most_circular)
			for c in contours[1:]:
				circularite = filtrage.get_circularite(most_circular)
				if(circularite>best_cirularity):
					best_cirularity = circularite
					most_circular = c


			(x,y,w,h) = cv2.boundingRect(most_circular)

		else:
			print("ONE SHAPE DETECTED")
			contour = contours[0]
			#croping around contour to save process time:
			(x,y,w,h) = cv2.boundingRect(contour)


		border = 100
		if(fast_algo):
			border=20
		imH,imW,_ = image_filtered.shape
		while(not (y-border > 0 and y+h+border< imH and x-border>0 and x+w+border<imW) ):
			print("image border too big, trying")
			border = border - 1

		image_filtered = image_filtered[y-border:y+h+border,x-border:x+y+border]
		offset = (x-border,y-border)



		assert image_filtered is not None, "empty image"
		return cv2.bitwise_not(image_filtered),offset,False

