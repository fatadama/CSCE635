## @file
# Control object for controlling EMILY

# import the definition of the gps class object
from xbee_bridge_state import gps_state
import math
# we plan on reading in numpy vectors - do I need to import numpy for this??
import numpy as np

headingThresholdDegrees = 20.0
headingThreshold = 20.0*math.pi/180.0
## range threshold: if closer than this value we assume we're at the target
rangeThreshold = 5.0

## Compute the heading error h1 - h2
def headingError(h1,h2):
    while h1 - h2 >= math.pi:
        h1 = h1 - 2.0*math.pi
    while h1 - h2 <= -math.pi:
        h1 = h1 + 2.0*math.pi
    return h1-h2

class control():
    def __init__(self):
        # allocate memory
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.psi = 0.0
        self.rangeRef = 0.0
        ## Target heading (rads)
        self.headingRef = 0.0
        ## target rudder setting
        self.rudder = 0.0
        ## target throttle setting
        self.throttle = 0.0
    ## Convert a commanded vector (2-length array) to range and heading and store
    def vectorRef(self,vector):
        self.rangeRef = math.sqrt( vector[0]*vector[0]+vector[1]*vector[1] )
        self.headingRef = math.atan2(vector[1],vector[0])
        return
    ## Convert an input gpsState to local x, y, v, and hdg
    #
    # @param[in] gpsState: a gps_state object that contains the current position in local coordinates
    # @param[in] v: the current velocity magnitude (m/s)
    # @oaram[in] psi: the current heading, in radians east of north
    def updateState(self,gpsState,v,psi):
        self.x = gpsState.x
        self.y = gpsState.y
        self.v = v
        self.psi = hdg
        return
    ## Update the control value based on the current state
    def update(self):
        # evaluate range to target. Do nothing if close
        if self.rangeRef < rangeThreshold:
            self.rudder = 0.0
            self.throttle = 0.0
            return
        deltaPsi = headingError(self.psi,self.headingRef)
        if deltaPsi > headingThreshold:# we are more than 20 degrees east of the target
            # turn left at a slow speed
            self.throttle = 0.2
            self.rudder = -1.0
        if deltaPsi < -headingThreshold: # we are more than 20 degrees west of the target
            # turn right at a slow speed
            self.throttle = 0.2
            self.rudder = 1.0
        else:# do PID on heading
            self.rudder = 0.0
            self.throttle = 0.0
        pass
