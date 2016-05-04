import cv2 as cv
import numpy as np

def HoughCircles(img):
	arr = []
	img = cv.medianBlur(img, 5)
	cimg = cv.cvtColor(img,cv.COLOR_GRAY2BGR)

	circles = cv.HoughCircles(img, cv.cv.CV_HOUGH_GRADIENT, 
				1, 20,param1=70,param2=50,minRadius=0, maxRadius=0)

	if(circles!=None):
		circles = np.uint16(np.around(circles))
		for i in circles[0,:]:
		    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
		    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
		    arr.append(i[0])
		    arr.append(i[1])
		    arr.append(i[2])
	arr.append(cimg)
	return arr

