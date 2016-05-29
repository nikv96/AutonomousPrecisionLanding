from dronekit import vehicle, connect 
import time
import math


def descend(vehicle, target, start_mission):
	from flightAssist import get_bearing
	from flightAssist import get_distance 


