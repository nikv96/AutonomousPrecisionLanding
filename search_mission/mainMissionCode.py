import time
import math
import threading
#importing thread pool to use to decipher thread return messages
from multiprocessing.pool import ThreadPool

import argparse  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not args.connect:
    print "Starting copter simulator (SITL)"
    from dronekit_sitl import SITL
    sitl = SITL()
    sitl.download('copter', '3.3', verbose=True)
    sitl_args = ['-I0', '--model', 'quad', '--home=-35.363261,149.165230,584,353']
    sitl.launch(sitl_args, await_ready=True, restart=True)
    connection_string = 'tcp:127.0.0.1:5760'


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

start_time = time.time()

#drop off location where drone is kept 
home_location = vehicle.location.global_relative_frame

#origin is taken as left vertex of rectangle in diagram
origin = get_location_metres(vehicle, home_location, 0, -110)
 
print "Taking off"

arm_takeoff(vehicle, 10)

#importing required functions
#TODO:need to figure out how to run these functions
import search_v1
import descend_v1
import Land_v1

tgt = search_v1.search()

tgt = LocationGlobalRelative(target.lat, target.lon, target_alt)
vehicle.simple_goto(tgt)
while vehicle.location.global_frame.alt != target_alt:
	print "Still landing"