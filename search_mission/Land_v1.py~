'''
The code ensures Landing by the drone once the drone has successfully identified the target

Landing Sequence - Dronekit land at coordinates followed by pinging from the shizzz.

PID controller is used to ensure that the drone stays on path.
'''

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import threading

#importing thread pool to use to decipher thread return messages
from multiprocessing.pool import ThreadPool

def land(vehicle, target):
	cur_alt = vehicle.location.global_frame[2]
	target_alt = vehicle.home_location[2]

	alt_diff = cur_alt - target_alt
	'''
	#for stationery precision landing
	home = vehicle.home_location
	home[0] = target[0]
	home[1] = target[1]

	while vehicle.home_location != home:
		vehicle.home_location = home
	
	vehicle.mode = VehicleMode("RTL")
	while (True):	
		if vehicle.mode.name == "RTL":
			break
		time.sleep(1)
	'''
	while vehicle.location.global_frame[2] != target_alt:
		tgt = LocationGlobalRelative(target[0], target[1], target_alt - 1)
		vehicle.simple_goto(tgt)
		#search()

def PIDController():
	



	
	
	
