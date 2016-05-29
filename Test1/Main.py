'''
This code connects the search, track and land and ensures precision landing.
'''
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from dronekit_sitl import SITL
import time
from pymavlink import mavutil
import search_img
import multiprocessing
from flightAssist import arm_and_takeoff
import control
import argparse
import sim
import video
import Queue
import cv2
import numpy as np

if __name__ == '__main__':	
	simulation = False
	parent_conn_im, child_conn_im = multiprocessing.Pipe()
	imageQueue = Queue.Queue()
	vehicleQueue = Queue.Queue()

	frame_count =0

	parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
	parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()
	connection_string = args.connect
	sitl = None

	#Start SITL if no connection string specified
	if not args.connect:
 	    simulation = True
	    print "Starting copter simulator (SITL)"
	    sitl = SITL()
	    sitl.download('copter', '3.3', verbose=True)
	    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
	    sitl.launch(sitl_args, await_ready=True, restart=True)
	    connection_string = 'tcp:127.0.0.1:5760'
	
	# Connect to the Vehicle
	print 'Connecting to vehicle on: %s' % connection_string
	veh_control = connect(connection_string, wait_ready=True)
	if simulation:
		print "Running simulation"
		sim.load_target('target.PNG')
		print "target loaded"
		sim.set_target_location(veh_control.location.global_relative_frame)
		print "target set"
	else:
		video.startCamera()

	arm_and_takeoff(veh_control,15)
	
	while True:
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

		img = multiprocessing.Process(name="img",target=search_img.analyze_frame, args = (child_conn_im, frame, location, attitude))
		img.daemon = True
		img.start()

		#retreive results
		results = parent_conn_im.recv()
		
		frame_count += 1
		
		# get image that was passed with the image processor
		img = imageQueue.get()
		#get vehicle position that was passed with the image processor
		location, attitude = vehicleQueue.get()
		#overlay gui
		rend_Image = search_img.add_target_highlights(img, results[3])
		#show/record images
		cv2.imshow("RAW", img)
		cv2.imshow("GUI", rend_Image)

		#send commands to autopilot
		control.land(veh_control, results,attitude,location)
		if veh_control.armed == False:
			break

	cv2.destroyAllWindows()
		
	print "Closing vehicle"
	veh_control.close()
	if sitl is not None:
		sitl.close()
