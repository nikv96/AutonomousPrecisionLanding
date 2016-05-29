from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import math
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
arm_and_takeoff(vehicle, 10)

targetAlt = 50

from flightAssist import send_ned_velocity
height = vehicle.location.global_relative_frame.alt


error = targetAlt - height
K_p = 7.5
K_i = 0.1
K_d = 2.5
accError = 0.0
errorPrev = 0.0

velocity_z = K_p*error + K_i*accError
send_ned_velocity(vehicle, 0, 0, -velocity_z, 0.1)

while True:
	print "Altitude: ", height
	
	height = vehicle.location.global_relative_frame.alt
	error = targetAlt - height

	accError += error*0.1
	dError = (error-errorPrev)/0.1

	velocity_z = K_p*error + K_i*accError + K_d*dError
	send_ned_velocity(vehicle, 0, 0, -velocity_z, 0.5)
	
	if abs(error) < 0.05:
		print "Reached target altitude!"
		send_ned_velocity(vehicle, 0, 0, 0, 1)
		break

	errorPrev = error	
	time.sleep(0.1)	 

vehicle.mode = VehicleMode("RTL")

while True:
	print "Altitude: ", vehicle.location.global_relative_frame.alt
	if vehicle.location.global_relative_frame.alt <= 0.5:
		print "Landed"
		break
	time.sleep(1)

vehicle.close()
