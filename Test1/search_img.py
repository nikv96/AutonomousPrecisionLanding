import urllib
import cv2
import numpy as np
import os
import math

def store_raw_images():
    #http://image-net.org/api/text/imagenet.synset.geturls?wnid=n00523513
    #http://image-net.org/api/text/imagenet.synset.geturls?wnid=n07942152
    neg_images_link = 'http://image-net.org/api/text/imagenet.synset.geturls?wnid=n00523513'   
    neg_image_urls = urllib.urlopen(neg_images_link).read().decode()
    pic_num = 828
    
    if not os.path.exists('neg'):
        os.makedirs('neg')
        
    for i in neg_image_urls.split('\n'):
        try:
            print(i)
            urllib.urlretrieve(i, "neg/"+str(pic_num)+".jpg")
            img = cv2.imread("neg/"+str(pic_num)+".jpg",cv2.IMREAD_GRAYSCALE)
            # should be larger than samples / pos pic (so we can place our image on it)
            resized_image = cv2.resize(img, (100, 100))
            cv2.imwrite("neg/"+str(pic_num)+".jpg",resized_image)
            pic_num += 1
            
        except Exception as e:
            print(str(e))

def create_pos_n_neg():
    for file_type in ['neg']:
        
        for img in os.listdir(file_type):

            if file_type == 'pos':
                line = file_type+'/'+img+' 1 0 0 50 50\n'
                with open('info.dat','a') as f:
                    f.write(line)
            elif file_type == 'neg':
                line = file_type+'/'+img+'\n'
                with open('bg.txt','a') as f:
                    f.write(line)

def get_target_pos():
	'''
	This function searches the video stream for the target and returns the position

	input: 
	output: x_angle and y_angle
	'''

	target_cascade = cv2.CascadeClassifier('target2.xml')

	'''
	Uncomment to test with image
	img = cv2.imread("Sample.jpg")

	gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

	target = target_cascade.detectMultiScale(gray, 1.1, 5)

	print target

	# add this
	for (x,y,w,h) in target:
		cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2)
	cv2.namedWindow("RESULT", cv2.WINDOW_NORMAL)
	cv2.imshow('RESULT',img)
	k = cv2.waitKey() & 0xff
	while not k == ord('q'):
		pass
	'''

	#Comment when using video sample
	cap = cv2.VideoCapture(0)

	#Uncomment to see video output
	#cv2.namedWindow("RESULT",cv2.WINDOW_NORMAL)

	i=0
	lock = True

	while True:
		ret, img = cap.read()
		if ret == False:
			print("Stream has ended")
			break
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

		#Modify the cars array every 25 frames
		if lock:
			target = target_cascade.detectMultiScale(gray,1.1,5)
			lock = not lock
		
		for (x,y,w,h) in target:
			cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,0),2)
		
		#Uncomment to see video output in gui
		#cv2.imshow('RESULT',img)
		k = cv2.waitKey(10) & 0xff
		i+=1
		if i==25:
			i=0
			lock = not lock
		if k == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()
	
	xcenter = x+w/2
	ycenter = y+h/2

	#shift origin to center of the image
	#NOTE: Change the pixels
	x_pixel = xcenter - (640.0/2.0)
	y_pixel = ycenter - (480.0/2.0)

    #convert target location to angular radians
	camera_hfov = 72.42
	camera_vfov = 43.3
	x_angle = x_pixel * (camera_hfov / 640) * (math.pi/180.0)
	y_angle = y_pixel * (camera_vfov / 480) * (math.pi/180.0)
	return (x_angle, y_angle)

def analyze_frame_async(child_conn):
	child_conn.send(get_target_pos())

if __name__ == "__main__":
	get_target_pos()
	#store_raw_images()
	#create_pos_n_neg()
