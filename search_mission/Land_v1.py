'''
The code ensures Landing by the drone once the drone has successfully identified the target

Landing Sequence - Dronekit land at coordinates followed by pinging from the shizzz.

PID controller is used to ensure that the drone stays on path.
'''

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import TargetAnalyzer
import time
import math
import threading

#importing thread pool to use to decipher thread return messages
from multiprocessing.pool import ThreadPool


#PID Controller

#Global Variables
p_gain = 0
i_gain = 0
d_gain = 0
imax = 0
integrator = 0
last_error = 0
last_update = 0

def PIDControllerInit(initial_p = 0, initial_i = 0, initial_d = 0, initial_imax = 0):
	'''
	Initializes all global variables
	'''
	p_gain = initial_p
	i_gain = initial_i
	d_gain = initial_d
	imax = abs(initial_imax)
	integrator = 0
	last_error = None
	last_update = time.time()

def get_dt(last_update, max_dt):
	now = time.time()
    time_diff = now - last_update
    last_update = now
    if time_diff > max_dt:
        return 0.0
    else:
        return time_diff

def get_p(error):
	return p_gain * error

def get_i(error, dt):
	integrator = integrator + error * i_gain * dt
	integrator = min(integrator, imax)
	integrator = max(integrator, -imax)
	return integrator

def get_d(error, dt):
	if last_error is None:
        last_error = error
    ret = (error - last_error) * d_gain * dt
    last_error = error
    return ret

def get_pi(error, dt):
	return get_p(error) + get_i(error, dt)

def get_pid(error, dt):
	return get_p(error) + get_i(error, dt) + get_d(error, dt)

def rst_integrator():
	integrator = 0


def get_new_a(x):
	return a - (x_old/x) * a

def get_new_v(x):
	return v - v_old


#Landing code sequence

def land(vehicle, target):
	'''
	Main landing code
	'''
	cur_alt = vehicle.location.global_frame.alt
	target_alt = vehicle.home_location.alt

	alt_diff = cur_alt - target_alt

	PIDControllerInit(1.0, 0.5, 0.01, 50)

	'''
	#for stationery precision landing
	home = vehicle.home_location
	home.lat = target.lat
	home.lon = target.lon

	while vehicle.home_location != home:
		vehicle.home_location = home
	
	vehicle.mode = VehicleMode("RTL")
	while (True):	
		if vehicle.mode.name == "RTL":
			break
		time.sleep(1)
	'''

	'''

	tgt = LocationGlobalRelative(target.lat, target.lon, target_alt)
	vehicle.simple_goto(tgt)
	while vehicle.location.global_frame.alt != target_alt:
		print "Still landing"
	return 
		#search()

	'''

	'''
	getimg()
	analyzeimg()
	apply pi and get acceleration
	convert acceleration into velocity
	move down
	'''

	while vehicle.location.global_frame.alt == vehicle.home_location.alt:
		tgt = search_img()
		target = LocationGlobalRelative(tgt[0], tgt[1], vehicle.home_location.alt)
		a_error_correction = get_pi(vehicle.location.global_frame.alt - target.alt, time.time())
		a = get_new_a(a_error_correction)
		x_old = a_error_correction
		v = get_new_v(a)
		#set velocity to negative v