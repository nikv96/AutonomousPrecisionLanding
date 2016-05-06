#from dronekit import connect, VehicleMode
import cv2
import numpy as np
import sys
from HoughLines import HoughLines
from HoughCircles import HoughCircles
import math

'''

img = cv2.imread('/home/nikhil/Documents/Python Projects/CIrcle and Line Detection/Landing Pad.PNG', 0)

houghCircleImage = HoughCircles(img)
houghLineImage = HoughLines(houghCircleImage)

cv2.imshow('Circles and Lines', houghLineImage)	

cv2.waitKey(0)
cv2.destroyAllWindows()
'''

#vehicle.connect('127.0.0.1:14550', wait_ready = True)

#real_radius = 1
#real_to_image_ratio = 0
cap = cv2.VideoCapture(0)

while(True):
	ret, frame = cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	houghCircleImage = HoughCircles(gray)
	#real_to_image_ratio = real_radius/houghCircleImage[2]
	#height = sample_height/real_to_image_ratio
	'''
	Get current position over here and determine center in real framework.
	

	Latitude = vehicle.location.global_frame.lat
	Longitude = vehicle.location.global_frame.lon
	
	center_x  = height/math.tan(theta) + 0.25 + real_radius	

	center_y = height/math.tan(theta) + 0.25 + real_radius 

	'''
	
	houghLineImage = HoughLines(houghCircleImage[len(houghCircleImage)-1])

	cv2.imshow('Circles and Lines', houghLineImage)	

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cv2.waitKey(0)
cv2.destroyAllWindows()

