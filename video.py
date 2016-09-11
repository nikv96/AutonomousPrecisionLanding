'''

    Synopsis: Script to obtain videos from webcam.
    Author: Nikhil Venkatesh
    Contact: mailto:nikv96@gmail.com

'''

#Opencv Imports
import cv2
import numpy as np

#Python Imports
import multiprocessing

cores_available = multiprocessing.cpu_count()

def image_capture_background(imgcap_connection):
	global cap, latest_image
        if imgcap_connection is None:
            print ("image_capture failed because pipe is uninitialised")
            return
	
        latest_image = None

        while True:
            success_flag, image = cap.read()
            if success_flag:
                latest_image = image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                if recv_obj == -1:
                    break
                imgcap_connection.send(latest_image)
        cap.release()

def startCamera():
	global cap, parent_conn, imgcap_conn, is_backgroundCap
	is_backgroundCap=False
	cap = cv2.VideoCapture(0)
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

	if not cap.isOpened():
		print("Cannot open camera")
		exit(0)

	if(cores_available > 3):
		print("BG process")
        parent_conn, imgcap_conn = multiprocessing.Pipe()
        proc = multiprocessing.Process(target=image_capture_background, args=(imgcap_conn,))
        proc.daemon = True
        proc.start()
        is_backgroundCap = True

	print("Camera is opened")

def get_frame():
    global cap, is_backgroundCap, parent_conn, img_counter
    img_counter =1
    if(is_backgroundCap):
        if(parent_conn == None):
            return None
        parent_conn.send(img_counter)
        img_counter = img_counter + 1
        img = parent_conn.recv()
    else:
        success_flag, img= cap.read()

    return img

def cap_end():
	global cap
	print("Releasing camera")
	cap.release()

if __name__ == "__main__":
	startCamera()
	i=1
	while True:
		img = get_frame()
		print("Got image " +str(i))
		if i==200:
			break
		i+=1
	cap_end()
