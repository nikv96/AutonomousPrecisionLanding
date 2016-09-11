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

current_milli_time = lambda: int(round(time.time() * 1000))

def analyze_frame(child_conn, img, location, attitude):
	start = current_milli_time()
	gray = img
	if(len(gray.shape) < 3):
		gray = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
	
	target = target_cascade.detectMultiScale(gray,1.1,5)
	if len(target)>0:
		center = (-1,-1)
		distance = -1
		for (x,y,w,h) in target:
			x_true = x + w/2.0 - hres/2.0
			y_true = -(y + h/2.0) + vres/2.0
			center = (x_true, y_true)
		stop = current_milli_time()
		child_conn.send((stop-start, center, target))
	else:
		stop = current_milli_time()
		child_conn.send((stop-start, None, None))

def add_target_highlights(image, target):
	img = copy(image)
	if(len(img.shape) < 3):
		img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

	if target is not None:
		for (x,y,w,h) in target:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
	return img

if __name__ == "__main__":
	print "In search_image"