import cv2
import numpy as np
import multiprocessing

cores_available = multiprocessing.cpu_count()

def image_capture_background(imgcap_connection):
	global cap, latest_image
        if imgcap_connection is None:
            print ("image_capture failed because pipe is uninitialised")
            return
	
        latest_image = None

        while True:
            # constantly get the image from the webcam
            success_flag, image = cap.read()

            # if successful overwrite our latest image
            if success_flag:
                latest_image = image

            # check if the parent wants the image
            if imgcap_connection.poll():
                recv_obj = imgcap_connection.recv()
                # if -1 is received we exit
                if recv_obj == -1:
                    break

                # otherwise we return the latest image
                imgcap_connection.send(latest_image)

        # release camera when exiting
        cap.release()

def startCamera():
	global cap, parent_conn, imgcap_conn, is_backgroundCap
	is_backgroundCap=False
	cap = cv2.VideoCapture(0)
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 640)
	cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 480)

	if not cap.isOpened():
		print "Cannot open camera"
		exit(0)

	if(cores_available > 3):
		print "BG process"
                # create pipe
                parent_conn, imgcap_conn = multiprocessing.Pipe()
                # create and start the sub process and pass it it's end of the pipe
                proc = multiprocessing.Process(target=image_capture_background, args=(imgcap_conn,))
                proc.daemon = True
                proc.start()

                #Mark that we are in background capture mode
                is_backgroundCap = True

	print "Camera is opened"

def get_frame():
	global cap, is_backgroundCap, parent_conn, img_counter
	img_counter =1
        if(is_backgroundCap):
            if parent_conn == None:
                return None

            parent_conn.send(img_counter)

            img_counter = img_counter + 1

            img = parent_conn.recv()

        else:
            success_flag, img= cap.read()

        return img

def cap_end():
	global cap
	print "Releasing camera"
	cap.release()

if __name__ == "__main__":
	startCamera()
	i=1
	while True:
		img = get_frame()
		print"Got image " +str(i)
		if i==200:
			break
		i+=1
	cap_end()
