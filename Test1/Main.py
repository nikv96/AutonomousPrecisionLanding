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

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
	parser.add_argument('--connect', 
		           help="Vehicle connection target string. If not specified, SITL automatically started and used.")
	args = parser.parse_args()
	connection_string = args.connect
	sitl = None
	#Start SITL if no connection string specified
	if not args.connect:
	    print "Starting copter simulator (SITL)"
	    sitl = SITL()
	    sitl.download('copter', '3.3', verbose=True)
	    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
	    sitl.launch(sitl_args, await_ready=True, restart=True)
	    connection_string = 'tcp:127.0.0.1:5760'
	# Connect to the Vehicle
	print 'Connecting to vehicle on: %s' % connection_string
	vehicle = connect(connection_string, wait_ready=True)
	parent_conn_im, child_conn_im = multiprocessing.Pipe()
	parent_conn_land, child_conn_land = multiprocessing.Pipe()
	land = multiprocessing.Process(name="land",target=control.main, args = (child_conn_land, vehicle,))
	land.daemon = True
	img = multiprocessing.Process(name="img",target=search_img.analyze_frame_async, args = (child_conn_im,))
	img.daemon = True
	land.start()
	img.start()
	l = ""
	while True:
		i = parent_conn_im.recv()
		l = parent_conn_land.recv()
		if l == "exit":
			break
		if i != None:
			parent_conn_land.send(i)
	print "Closing vehicle"
	vehicle.close()
	sitl.close()
