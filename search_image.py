'''

    Synopsis: Script to analyze frames for presence of target.
    Author: Nikhil Venkatesh
    Contact: mailto:nikv96@gmail.com

'''

#Python Imports
import urllib
import os
import math
import time
from copy import copy

#Opencv Imports
import cv2
import numpy as np

#Global Variables
hres = 640
vres = 480
vfov = 48.7
hfov = 49.7
target_cascade = cv2.CascadeClassifier(os.path.dirname(os.path.realpath(__file__))+"/Resources/target_cascade.xml")
if target_cascade.empty():
	exit()

def is_cv2():
    return check_opencv_version("2.",)
 
def is_cv3():
    return check_opencv_version("3.")
 
def check_opencv_version(major):
    return cv2.__version__.startswith(major)

current_milli_time = lambda: int(round(time.time() * 1000))

def analyze_frame(child_conn, img, location, attitude):
	t1 = time.time()
	frame = img
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	color = cv2.inRange(hsv,np.array([100,50,50]),np.array([140,255,255]))
	image_mask=color
	element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
	image_mask = cv2.erode(image_mask,element, iterations=2)
	image_mask = cv2.dilate(image_mask,element,iterations=2)
	image_mask = cv2.erode(image_mask,element)
	if is_cv2():
		contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	else:
		_, contours, hierarchy = cv2.findContours(image_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	maximumArea = 0
	bestContour = None
	for contour in contours:
		currentArea = cv2.contourArea(contour)
		if currentArea > maximumArea:
			bestContour = contour
			maximumArea = currentArea
	if bestContour is not None:
		x,y,w,h = cv2.boundingRect(bestContour)
		center = (x + w/2.0 - hres/2.0, -(y + h/2.0) + vres/2.0)
		t2 = time.time()
		child_conn.send((t2-t1, center, (x,y,w,h)))
	else:
		t2 = time.time()
		child_conn.send((t2-t1, None, None))

def add_target_highlights(image, target):
	img = copy(image)
	if(len(img.shape) < 3):
		img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

	if target is not None:
		x, y, w, h = target
		cv2.rectangle(img, (x,y),(x+w,y+h), (0,0,255), 3)
	return img

if __name__ == "__main__":
	print "In search_image"