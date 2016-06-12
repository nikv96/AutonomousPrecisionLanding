from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
from pymavlink import mavutil
import argparse
from flightAssist import send_ned_velocity
from position_vector import PositionVector
import pid
import sim
import math

abort_height = 30
search_attempts = 200
attempts = 0
climb_altitude = 10
last_valid_target = None
target_detected = False
valid_target = False
initial_descent = True
climbing = False
simulator = True

x_pid = pid.pid(5.0, 2.0, 1.0, 50)
y_pid = pid.pid(5.0, 2.0, 1.0, 50)

def land(veh_control, target_info,attitude,location):
	global last_valid_target, target_detected, valid_target, initial_descent, climbing, attempts
	valid_target = False
	now = time.time()
	
	if target_info[1] is not None:
		target_detected = True
		valid_target = True
		initial_descent = False
		last_valid_target = now

	if(inside_landing_area(veh_control) == 1):
		if(veh_control.location.global_relative_frame.alt <2):
			veh_control.mode = VehicleMode("LAND")
		if(target_detected):
			climbing = False
			initial_descent = False

			if(valid_target):
				move_to_target(veh_control,target_info,attitude,location)

			else:
				if(now - last_valid_target > 1.5):
					target_detected = False

				if(veh_control.location.global_relative_frame.alt > abort_height):
					straight_descent(veh_control)
				else:
					send_ned_velocity(veh_control,0,0,0,0.1)

		#there is no known target in landing area
		else:
			if(climbing):
				climb(veh_control)

			#not searching, decide next move
			else:	
				#top section of cylinder
				if(veh_control.location.global_relative_frame.alt > abort_height):
					#initial descent entering cylinder
					if(initial_descent):
						autopilot_land(veh_control)

					#all other attempts prior to intial target detection
					else:
						straight_descent(veh_control)

				#lower section of cylinder
				else:
					#we can attempt another land
					if(attempts < search_attempts):
						attempts += 1
						climb(veh_control)

					else:
						autopilot_land(veh_control)


	elif(inside_landing_area(veh_control) == -1):
		straight_descent(veh_control)
		target_detected = False

	else:
		autopilot_land(veh_control)
		target_detected = False
		initial_descent = True

def shift_to_origin(pt,width,height):
	return ((pt[0] - width/2.0),(-1*pt[1] + height/2.0))

def pixel_point_to_position_xy(pixel_position,distance):
		thetaX = pixel_position[0] * 60 / 640
		thetaY = pixel_position[1] * 60 / 480
		x = distance * math.tan(math.radians(thetaX))
		y = distance * math.tan(math.radians(thetaY))

		return (x,y)

#move_to_target - fly aircraft to landing pad
def move_to_target(veh_control,target_info,attitude,location):
	x,y = target_info[1]
	
	x,y = shift_to_origin((x,y), 640, 480)
	
	if(simulator):
		hfov = 60
		vfov = 60
	else:
		hfov = 60
		vfov = 60

	x -= (640 / hfov) * math.degrees(attitude.roll)
	y += (480 / vfov) * math.degrees(attitude.pitch)

	X, Y = pixel_point_to_position_xy((x,y),location.alt)

	target_heading = math.atan2(Y,X) % (2*math.pi)
	target_heading = (attitude.yaw - target_heading) 
	target_distance = math.sqrt(X**2 + Y**2)

	speedx = target_distance * math.sin(target_heading) 
	speedx = x_pid.get_pid(speedx, 0.1)
	speedy = target_distance * math.cos(target_heading)
	speedy = y_pid.get_pid(speedy, 0.1)

	vx = speedx * -1
	vy = speedy 

	if(target_distance > 1):
		vz = 0
	else:
		vz = 2.5
	send_ned_velocity(veh_control,vx,vy,vz,0.1)

#autopilot_land - Let the autopilot execute its normal landing procedure
def autopilot_land(veh_control):
	#descend velocity
	send_ned_velocity(veh_control,0,0,0.5,0.1)
	#veh_control.set_velocity(9999,9999,9999)

#straight_descent - send the vehicle straight down
def straight_descent(veh_control):
	send_ned_velocity(veh_control,0,0,0.5,0.1)

#climb - climb to a certain alitude then stop.
def climb(veh_control):
	global climbing, climb_altitude
	if(veh_control.location.global_relative_frame.alt < climb_altitude):
		send_ned_velocity(veh_control,0,0,-0.5,0.1)
		climbing = True
	else:
		send_ned_velocity(veh_control,0,0,0,0.1)
		climbing = False


#inside_landing_area - determine is we are in a landing zone 0 = False, 1 = True, -1 = below the zone
def inside_landing_area(veh_control):

	vehPos = PositionVector.get_from_location(veh_control.location.global_relative_frame)
	landPos = sim.targetLocation
	if(PositionVector.get_distance_xy(vehPos,landPos) < 20):
		#below area
		if(vehPos.z < 0.5):
			return -1
		#in area
		else:
			return 1
	#outside area
	else:
		return 0

if __name__ == "__main__":
	main()
