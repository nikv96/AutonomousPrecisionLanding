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

#Python Imports
import math
import time
import argparse

#Global Variables
x_pid = pid.pid(0.1, 0.005, 0.1, 50)
y_pid = pid.pid(0.1, 0.005, 0.1, 50)
z_pid = pid.pid(0.2, 0.005, 0.1, 50)
hfov = 60
hres = 640
vfov = 60
vres = 480
x_pre = 0
y_pre = 0

def get_cam_pitch():
	return 0

def land(vehicle, target, attitude, location):
	if(vehicle.location.global_relative_frame.alt <= 2.7):
		vehicle.mode = VehicleMode('LAND')
	if(target is not None):
		move_to_target(vehicle,target,attitude,location)
	elif(vehicle.location.global_relative_frame.alt > 30):
		vehicle.mode = VehicleMode('LAND')
	else:
		send_velocity(vehicle, 0, 0, -0.5, 1)
		
def move_to_target(vehicle,target,attitude,location):
	x,y = target

	alt = vehicle.location.global_relative_frame.alt

	pitch_cam = math.radians(get_cam_pitch())

	X = alt * math.tan( pitch_cam + math.atan( (x * math.tan( math.radians(hfov/2) ) )/ (hres/2) ) )
	Y = alt * math.tan( pitch_cam + math.atan( (x * math.tan( math.radians(vfov/2) ) )/ (vres/2) ) )

	vx = x_pid.get_pid(X, 0.1)
	vy = -y_pid.get_pid(Y, 0.1)
	
	print("x = " + str(X))
	print("vx = " + str(vx))
	print("y = " + str(Y))
	print("vy = " + str(vy))

	if(math.sqrt(X**2 + Y**2) > 2):
		vz = 0
	else:
		vz = z_pid.get_pid(alt, 0.1)
	send_velocity(vehicle, vy, vx, vz, 1)