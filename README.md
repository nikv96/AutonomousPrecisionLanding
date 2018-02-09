# AutonomousPrecisionLanding
Autonomously landing a drone on a visual target. This project was built to prototype an algorithm for the [Mohammad Bin Zayed International Robotics Competition 2017](http://mbzirc.com/).

The project makes use of the dronekit-python library to connect to a Pixhawk based drone. Dronekit-sitl is used to validate the algorithm in a simulated environment. The target tracking is performed using the Viola Jones approach for Rapid Object Detection using HAAR-cascades.

![alt text](https://github.com/nikv96/AutonomousPrecisionLanding/blob/master/Logs/Target%20Tracking.png)

## Install Dependencies
1. Install Python 2.7 [here](https://www.python.org/download/releases/2.7/).
2. Install OpenCV-python 3.1.0 from [here](http://opencv.org/downloads.html).
3. Install Dronekit from [here](http://python.dronekit.io/guide/quick_start.html).

## How To Run
_Create a directory called `Vids/` in Logs/_
1. Run the code with ```python main.py``` to run on simulation.
2. Run the code with ```python main.py --connect <connection_string>``` to run on the drone.

## Contributors
1. [Nikhil Venkatesh](https://github.com/nikv96)
2. [Rahul Nambiar](https://github.com/RNambiar1996)

## Acknowledgements
1. Daniel Nugent's work on precision landing - https://github.com/djnugent/Precland.
2. Viola and Jones' paper - Rapid Object Detection using a Boosted Cascade of Simple
Features.

**Disclaimer - This repository is no longer maintained.**
