'''

	Synopsis: Script to run the control algorithm.
	Author: Nikhil Venkatesh
	Contact: mailto:nikv96@gmail.com

'''

#Dronekit Imports
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil

#Common Library Imports
from flight_assist import send_velocity
from position_vector import PositionVector
import pid
import sim
#import epm

#Python Imports
import math
import time
import argparse

#Global Variables
x_pid = pid.pid(0.1, 0.005, 0.1, 50)
y_pid = pid.pid(0.1, 0.005, 0.1, 50)
hfov = 60
hres = 640
vfov = 60
vres = 480
x_pre = 0
y_pre = 0
simulation = False
if simulation:
	epm_object = epm.EPM()

def pixels_per_meter(fov, res, alt):
	return ( ( alt * math.tan(math.radians(fov/2)) ) / (res/2) )

def land(vehicle, target, attitude, location):
	if(vehicle.location.global_relative_frame.alt <= 0.15):
		send_velocity(vehicle, 0, 0, 0, 1)
		time.sleep(10)
		vehicle.mode = VehicleMode('RTL')
		return
	elif(vehicle.location.global_relative_frame.alt <= 0.3):
		if simulation:
			epm_object.on()
		return
	if(target is not None):
		move_to_target(vehicle,target,attitude,location)
	elif(vehicle.location.global_relative_frame.alt > 30):
		vehicle.mode = VehicleMode('RTL')
	else:
		send_velocity(vehicle, 0, 0, -0.25, 1)
		
def move_to_target(vehicle,target,attitude,location):
	x,y = target

	alt = vehicle.location.global_relative_frame.alt
	px_meter_x = pixels_per_meter(hfov, hres, alt)
	px_meter_y = pixels_per_meter(vfov, vres, alt)

	x *= px_meter_x
	y *= px_meter_y

	vx = x_pid.get_pid(x, 0.1)
	vy = y_pid.get_pid(y, 0.1)
	
	print("x = " + str(x))
	print("vx = " + str(vx))
	print("y = " + str(y))
	print("vy = " + str(vy))

	if(math.sqrt(x**2 + y**2) > 2):
		vz = 0
	else:
		vz = 0.2
	send_velocity(vehicle, vy, vx, vz, 1)