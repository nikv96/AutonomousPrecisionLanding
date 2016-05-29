from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import math

def flyTo(r_0, dY, dX, dZ):
	  
	#+ dY forward 
	#+ dX to the right
	#+ dZ up   
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
		0b0000111111000111, # type_mask (only speeds enabled)
		dX, dY, dZ, # x, y, z positions (not used)
		0, 0, 0, # x, y, z velocity in m/s
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	

	vehicle.send_mavlink(msg)

def xy_to_latlon(r_0, dY, dX):
	
	theta = vehicle.heading
	
	dNorth = 0.0
	dEast = 0.0

	dL = math.sqrt(dY*dY + dX*dX)
		
	alpha = math.atan2(dY, dX)
	
	beta = alpha - theta

	dNorth = dL*math.sin(beta)
	dEast = dL*math.cos(beta)
	
	from flightAssist import get_location_metres
	target = get_location_metres(vehicle, r_0, dNorth, dEast)

	return target
	
#these velocity breakdowsns give north and east velocities to give effective
#velocities forward/backward and right/left

#return velNorth 
def vel_north(airspeed, dY, dX):
	theta = vehicle.heading
	alpha = math.atan2(dY, dX)
	beta = alpha - theta

	velNorth = airspeed*math.sin(beta)
	return velNorth
		

#return velEast 
def vel_east(airspeed, dY, dX):
	theta = vehicle.heading
	alpha = math.atan2(dY, dX)
	beta = alpha - theta

	velEast = airspeed*math.cos(beta)
	return velEast

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

from flightAssist import arm_and_takeoff
arm_and_takeoff(vehicle, 20)

r_0 = vehicle.location.global_relative_frame
target = xy_to_latlon(r_0, 1000, 0)
vehicle.airspeed = 10

from flightAssist import get_distance_metres
distance = get_distance_metres(vehicle.location.global_relative_frame, r_0, target)

vehicle.simple_goto(target)

while distance > 1:
	print "Position: ", vehicle.location.global_relative_frame
	print "Distance to target: ", distance 
	distance = get_distance_metres(vehicle, vehicle.location.global_relative_frame, target)	

currentLocation = vehicle.location.global_relative_frame
target = xy_to_latlon(currentLocation, 0, 1000)
vehicle.airspeed = 10

from flightAssist import get_distance_metres
distance = get_distance_metres(vehicle.location.global_relative_frame, r_0, target)

vehicle.simple_goto(target)

while distance > 1:
	print "Position: ", vehicle.location.global_relative_frame
	print "Distance to target: ", distance 
	distance = get_distance_metres(vehicle, vehicle.location.global_relative_frame, target)	


vehicle.mode = VehicleMode("RTL")

vehicle.close()



	

