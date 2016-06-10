import urllib
import cv2
import numpy as np
import os
import math
import time
from copy import copy

cam_width = 640
cam_height = 480
cam_vfov = 48.7
cam_hfov = 49.7
target_cascade = cv2.CascadeClassifier(os.path.dirname(os.path.realpath(__file__))+"/target2.xml")
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
		center =(-1,-1)
		for (x,y,w,h) in target:
			center = (x+w/2,y+h/2)
	
		#shift origin to center of the image
	        x_pixel = center[0] - (cam_width/2.0)
	        y_pixel = center[1] - (cam_height/2.0)

	        #convert target location to angular radians
	        x_angle = x_pixel * (cam_hfov / cam_width) * (math.pi/180.0)
	        y_angle = y_pixel * (cam_vfov / cam_height) * (math.pi/180.0)

		x_pixel -= (cam_width / cam_hfov) * math.degrees(attitude.roll)
		y_pixel += (cam_height / cam_vfov) * math.degrees(attitude.pitch)

		thetaX = x_pixel * cam_hfov / cam_width
		thetaY = y_pixel * cam_vfov / cam_height
		x = location.alt * math.tan(math.radians(thetaX))
		y = location.alt * math.tan(math.radians(thetaY))
		target_heading = math.atan2(y,x) % (2*math.pi)
		target_heading = (attitude.yaw - target_heading) 
		distance = math.sqrt(x**2 + y**2)
		stop = current_milli_time()
		child_conn.send((stop-start, center, distance, target))
	else:
		stop = current_milli_time()
		child_conn.send((stop-start,None, -1, None))

def add_target_highlights(image, target):
	#create a shallow copy of image
	img = copy(image)
	if(len(img.shape) < 3):
		img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

	if target is not None:
		for (x,y,w,h) in target:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
	return img

if __name__ == "__main__":
	print "In search_img"
