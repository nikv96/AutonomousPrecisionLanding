from dronekit import connect

from ..Searching import Searching as S
from ..Searching/ImageProcessingTools import TargetDetect as I
from ..Tracking import Tracking as T
from ..Landing import Landing as L

from Threading import thread

'''
This code connects the search, track and land and ensures precision landing.
'''


