from dronekit import connect

#connect to the vehicle
vehicle = connect("127.0.0.1:14550", wait_ready=True)

'''
This code takes the input from the search code and begins the tracking session while landing. Tracking is also accompanied with pinging of the image processing code to detect changes in position.
'''

def track():
	
