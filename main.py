'''

	Synopsis: This is the main control thread that synchronizes the control code and the image processing code.
	Author: Nikhil Venkatesh
	Contact: mailto:nikv96@gmail.com

'''

# Dronekit imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from dronekit_sitl import SITL
from pymavlink import mavutil

# Helper Libraries Imports
import search_image
import multiprocessing
from flight_assist import arm_and_takeoff
import control
import sim
import video

# Opencv Imports
import cv2
import numpy as np

# Python Imports
import time
import argparse
import Queue
import os

if __name__ == '__main__':	
	simulation = False
	sitl = None
	parent_conn_im, child_conn_im = multiprocessing.Pipe()
	imageQueue = Queue.Queue()
	vehicleQueue = Queue.Queue()
	frame_count =0

	parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
	parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()
	connection_string = args.connect
	
	if not args.connect:
 	    simulation = True
	    print "Starting copter simulator (SITL)"
	    sitl = SITL()
	    sitl.download('copter', '3.3', verbose=True)
	    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
	    sitl.launch(sitl_args, await_ready=True, restart=True)
	    connection_string = 'tcp:127.0.0.1:5760'
	
	print("Connecting to vehicle on: %s" % connection_string)
	veh_control = connect(connection_string, wait_ready=True, baud=57600)
	
	if simulation:
		veh_control.mode = VehicleMode("GUIDED")

	while True:
		if(veh_control.mode == "GUIDED"):
			break
		time.sleep(0.1)
	if(simulation):
		print("Running simulation...")
		sim.load_target('Resources/target.PNG')
		print("Target loaded.")
		target = LocationGlobalRelative(veh_control.location.global_relative_frame.lat+0.00002, veh_control.location.global_relative_frame.lon - 0.00002, veh_control.location.global_relative_frame.alt)
		sim.set_target_location(target)
		print("Target set.")
		arm_and_takeoff(veh_control, 10)
	else:
		video.startCamera()

	fourcc = cv2.cv.CV_FOURCC(*'XVID')
	i = len([name for name in os.listdir('Logs/Vids')])
	vid = cv2.VideoWriter('Logs/Vids/log'+str(i)+'.avi', fourcc, 30.0, (640, 480))

	
	while True:
		if not (veh_control.mode == "GUIDED"):
			if simulation:
				break
			else:
				continue
		if not (veh_control.armed):
			break
		location = veh_control.location.global_relative_frame
		attitude = veh_control.attitude
		print "Altitude =" + str(veh_control.location.global_relative_frame.alt)
		
		if simulation:
			sim.refresh_simulator(location,attitude)
			frame = sim.get_frame(attitude)
			cv2.waitKey(1)
		else:
			frame = video.get_frame()

		imageQueue.put(frame)
		vehicleQueue.put((location,attitude))

		img = multiprocessing.Process(name="img",target=search_image.analyze_frame, args = (child_conn_im, frame, location, attitude))
		img.daemon = True
		img.start()

		results = parent_conn_im.recv()
		
		frame_count += 1
		
		img = imageQueue.get()
		location, attitude = vehicleQueue.get()
		rend_Image = search_image.add_target_highlights(img, results[3])
		
		if simulation:
			cv2.imshow("RAW", img)
			cv2.imshow("GUI", rend_Image)
		
		vid.write(rend_Image)

		control.land(veh_control, results,attitude,location)
		time.sleep(0.1)

	vid.release()
		
	print("Closing vehicle")
	veh_control.close()
	if sitl is not None:
		sitl.stop()
