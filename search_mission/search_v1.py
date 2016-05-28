from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import threading
#importing thread pool to use to decipher thread return messages
from multiprocessing.pool import ThreadPool

from /../TargetAnalyzer import search_img

#states to define different scenarios 
#State 1 for initial search routine
#State 2 to search when target lost at destination altitude
#State 3 if cannot find target at all for certain time interval
#Time interval to be defined

#need to add function to initiate cameras and servos

def look_for_target(event):
	start_time_2 = time.time()	
	while True:
		#image processing code here
		result = search_img.get_target_pos()
		#dummy code
		if time.time()-start_time_2 > 3*60:
			event.set()

def set_angle(property, value):	
 	try:
		f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
		f.write(str(angle))
	except:
		print("Error writing to: " + property + " value: " + value)

def setServo(property,maxAngle,event):
			
	#defining constants 
	delay_period = 0.5
	del_theta = 5
	angle = 0 
	
	#loop to turn servo
	#returns true if target is found
	while angle<maxAngle:	
		set_angle("servo", angle)		
		angle+=del_theta
		if event.isSet():
			return True
		time.sleep(delay_period)


def search(vehice, state, home, start_mission, lastDetectedPos, timeLastDetected):
		
	#setting start time as t0	
	start_search = time.time()

	from flightAssist import get_location
	origin = get_location_metres(vehicle, home, 0, -110)

	#the four corners of the rectangular box
	bLeft = get_location_metres(vehicle, 0, 10)
	bLeft.alt = maxAlt

	bRight = get_location_metres(vehicle, home, 0, -20)
	bRight.alt = maxAlt

	tLeft = get_location_metres(vehicle, home, 60, -100)
	rLefta.alt = maxAlt

	tRight = get_location_metres(vehicle, home, 60, -20)	
	tRight.alt = maxAlt
	
	#Maximum altitude to look for target		
	maxAlt = 50
	altSteps = 5
		
	event = threading.Event()
		
	#initiating servo	
	set("delayed", "0")
	set("mode", "servo")
	set("servo_max", "180")
	set("active", "1")
	
	#initiating camera 	
	#cam = camera()
	#cam.start()
	
	#adding servos to control position of camera
	#servo = Servo()

	#threshold time to stay in state 1
	min_threshold = 2.5
	
	#parameter to check if drone has moved to a corner
	atCorner = False
	
	#defining thread to look for target	
	i1 = threading.Thread(name="find",target = look_for_target, args = (event, ))
	i1.start()
	
	#loop to continue searching until object found
	while True:
		
		#initial search routine
		if state == 1:
			targetAlt = maxAlt
			#launching vehicle to max altitude
			vehicle.simple_goto(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon, maxAlt)

			while True:
				if vehicle.location.global_relative_frame.alt >= 0.95*targetAlt:
					print "At target altitude"
					break
				
				
				#Execute loop as target is not found
				while True:
					if rotateServos("servo",180, event) == True:
						print "Target found"						
						return True	
					
					#calculating elapsed time since search was started	
					elapsed_time = time.time() - start_search
	
					if elapsed_time >= min_threshold*60:
						print "2.5 minutes have elapsed. Exiting state 1"
						state = 4
						break
					
		#state 2 mission routine
		#search routine when target is lost during descent or landing phase
		#and could not be found after going to predicted location
		#also to be used when lost during landing
		elif state == 2:			

			z = vehicle.location.global_relative_frame.alt
			x = vehicle.location.global_relative_frame.lat
			y = vehicle.location.global_relative_frame.lon
			
			#increasing drone height to 
			vehicle.simple_goto(x,y,z)
				
			#loop to ensure vehicle reaches required height			
			while True:
				print "Altitude: ", z
				if vehicle.location.global_relative_frame.alt>=0.95*z:
					print "At ", z
					break		

			#time delay to give sufficient time for servos to rotate 360 degrees		
			if rotateServos("servo, 180, event") == True:
				print "Target found"
				return True

			z += altSteps
			
			#switch to state 4 if 
			if z >= 0.95*maxAlt:					
				state = 4
			else :
				state = 2
		
		#to predict location of target when lost during descent
		elif state == 3:
			#here need to predict location of target when lost during descent 
			#from last detected position/time last spotted/time since mission began
			from flightAssist import approximateLocation
			#need to determine rotation here 
			rotation = 1

			#predicting the position of the target after certain time
			predictedPos = approximateLocation(vehicle, timeLastDetected, lastDetectedPos, origin, start_mission, rotation)
			
			targetHeight = 5
			predictedPos.alt = targetHeight


			vehicle.simple_goto(predictedPos)

			while True:
				currentAltitude = vehicle.location.global_relative_frame.alt
				print "Altitude: ", currentAltitude 
				if currentAltitude < 1.01*targetHeight:
					break

			#switch to state 2 after descending to required height
			state = 2
			
			
		#state 4 seerch routine to move in a rectangular path
		elif state == 4:
			#set camera to point inwards 
			set_angle("servo", 90)
		
			#check which box corner is closest
			current_location = vehicle.location.global_relative_frame	
			from flightAssist import get_distance_metres

			if atCorner == False:
				#distance from 	bottom left	
				d_bl = get_distance_metres(vehicle, currentLocation,b_Left)
				#distance from bottom right
				d_br = get_distance_metres(vehicle, current_location, b_Right)
				#distance from top left
				d_tl = get_distance_metres(vehicle, current_location, t_Left)
				#distance from top right
				d_tr = get_distance_metres(vehicle, current_location, t_Right)
				
				shortest_distance = d_bl
				if d_br <= shortest_distance:
					shortest_distance = d_br

				elif d_tl <= shortest_distance:	
					shortest_distance = d_tl

				elif d_tr <= shortest_distance
					shortest_distance = d_tr

				target = LocationGlobalRelative(0,0,0)
				
				switch(shortest_distance):
					case d_bl: 
						target = bLeft
						break
					case d_br:
						target = bRight
						break
					case d_tl:
						target = tLeft
						break
					case d_tr:
						target =tRight
						break
			

				vehicle.simple_goto(target)
				while True:
					distance = get_distance_metres(vehicle, vehicle.location.global_relatvive_frame, target)
					print "Distance; ", distance
					
					#checking to see if image processing 
					#thread has found target 					
					if event.isSet():
						return True
					if distance < 1:
						print "Reached corner " 
						break

				#setting flag to indicate that drone has already moved to corner
				atCorner = True

			#once the drone has already moved to a corner		
			else:
				#if at the left edge of the rectangle
				if abs(currentLocation.lon - bLeft.lon) < epsilon and abs(current_location.lat - tLeft.lat) > epsilon:
					velocity_x = 0
					velocity_y = 5
					
				#if at the top edge
				elif abs(currentLocation.lat - tLeft.lat) < epsilon and abs(current_location.lon - tRight.lon) > epsilon:
					velocity_x = 5
					velocity_y = 0
						
				#if at the right edge
				elif abs(currentLocation.lon-tRight.lon) < epsilon and abs(currentLocation.lat-bRight.lat) > epsilon:
					velocity_x = 0
					velocity_y = -5
					
				#if at the bottom edge
				elif abs(currentLocation.lat-bRight.lat) < epsilon and abs(currentLocation.lon-bLeft.lon)> epsilon:				
					velocity_x = -5
					velocity_y = 0

				velocity_z = 0
				
				if event.isSet():
					return True

				from flightAssist import send_ned_velocity
				send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z, 2)
	
	
					

	
