from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
from pymavlink import mavutil
from flightAssist import get_location_metres
from flightAssist import get_distance_metres
import argparse  

def download_mission(vehicle):
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete

def adds_square_mission(vehicle, aLocation, aSize):
    cmds = vehicle.commands
    print " Clear any existing commands"
    cmds.clear() 
    
    print " Define/add new commands."
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(vehicle, aLocation, aSize, -aSize)
    point2 = get_location_metres(vehicle, aLocation, aSize, aSize)
    point3 = get_location_metres(vehicle, aLocation, -aSize, aSize)
    point4 = get_location_metres(vehicle, aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    
    print " Upload new commands to vehicle"
    cmds.upload()

def distance_to_current_waypoint(vehicle):

    """

    Gets distance in metres to the current waypoint. 

    It returns None for the first waypoint (Home location).
    """

    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None

    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle, vehicle.location.global_frame, targetWaypointLocation)

    return distancetopoint

def main(c, vehicle):
    #initial location of target 
    initial_location = vehicle.location.global_frame
    #adding mission to move in a square path
    adds_square_mission(vehicle, initial_location, 2.5)

    arm_and_takeoff(vehicle, 5)
    
    vehicle.commands.next = 0
    vehicle.mode = VehicleMode("AUTO")
    
    start_time = time.time()
    i = ""
    
    while True:
        i = c.recv()        
        if i != None:
            vehicle.home_location = LocationGlobalRelative(i[0], i[1], vehicle.home_location.alt)
            break
    	nextWayPoint = vehicle.commands.next
    	print "Distance to next waypoint: ", distance_to_current_waypoint(vehicle)

    	if nextWayPoint == 3:
    		vehiclecommands.next = 5
    	elif nextWayPoint == 5:
    		vehicle.commands.next = 0
    	if time.time()-start_time > 10:
    		break
    time.sleep(2)
    
    print "Mission completed" 
    vehicle.mode = VehicleMode("RTL")
    
    while True:
    	h = vehicle.location.global_frame.alt
    	print "Landing...."
    	print "Altitude: ", h

    	if h<0.1:
    		print "Landed"
    		break

    c.send("exit")