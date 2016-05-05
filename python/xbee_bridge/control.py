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
    def __init__(self,logdir=None,Kp=0.0,Kd=0.0,Ki=0.0):
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
        ## PID object for heading commands
        self.headingPid = PIDclass(Kp,Ki,Kd,logDir=logDir,name='hdg',umin=-1.0,umax=1.0)
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
        else:# TODO do PID on heading
            self.rudder = 0.0
            self.throttle = 0.0

def defaultDifference(x,r):
    return x-r

## PID class from old Crazyflie code. For purely SISO systems.
class PIDclass:
    def __init__(self,Kp,Ki,Kd,logDir = None,name = 'pid1',umax = 1e9,umin = -1e9):
        # last value of state, X
        self.stateLast = 0.0
        # last systime time of update - used to tell if we've lost data
        self.sysTimeLast = time.time()
        # last timestamp
        self.timeLast = 0.0
        # current P, I, D state values
        self.pidstate = np.array([0.0,0.0,0.0])
        # PID gain array
        self.pidarray = np.array([Kp,Ki,Kd])
        # store control copy locally
        self.u = np.array([0.0,0.0,0.0])
        # flag that says if we've received data before - don't use integral term until this has been set
        # otherwise we get a huge jump in delta-time and integral term explodes
        self.flag_init = False
        # control limits (use arbitrary large numbers)
        self.u_max = umax
        self.u_min = umin
        # log file
        if logDir is not None:
            #open log file
            self.fid = open(logDir + ('pid_%s.log' % (name)),'w')
            self.fid.write("time (ms),x,ref,state_p,state_i,state_d,u\n")
            print("Opened PID log file for state %s" % name)
        else:
            self.fid = None
    ## reset object to its initial state
    def reset(self):
        self.pidstate[0] = 0.0
        self.pidstate[1] = 0.0
        self.pidstate[2] = 0.0
        self.flag_init = False
        self.sysTimeLast = time.time()
        self.stateLast = 0.0
        self.u[0] = 0.0
        self.u[1] = 0.0
        self.u[2] = 0.0
        return
    ## Update - receive a new measurement, compute the control output
    # @param[in] t the current time, used to compute derivative and integral terms
    # @param[in] x the current state
    # @param[in] r the current reference or goal value
    # @param[in] diffFun function name to use to compute the error - option to pass in a custom function for things like attitude (heading) errors. Format: e = differenceFunction(x,r)
    def update(self,t,x,r = 0.0,diffFun=defaultDifference):
        # proportional term
        prop = diffFun(x,r)
        if self.flag_init:
            # update derivative
            if (t != self.timeLast):
                deri = (prop-self.stateLast)/(t-self.timeLast)
            else:
                deri = 0.0# divide by zero protection
            #increment integral
            inte = (t-self.timeLast)*(x-r)
        else:
            deri = 0.0
            inte = 0.0
            self.flag_init = True
        # compute the P,I,D components
        self.pidstate[0] = prop
        self.pidstate[1] = self.pidstate[1] + inte
        self.pidstate[2] = deri
        # compute control
        uc = -1.0*np.dot(self.pidarray,self.pidstate)
        # control limits
        uc = max(self.u_min,min(uc,self.u_max))
        # save current system time
        self.sysTimeLast = time.time()
        # save timestamp
        self.timeLast = t
        # save last state
        self.stateLast = prop
        # save control locally
        self.u = uc
        # log control
        if self.fid is not None:
            # write time, x, r, prop, inte, deri, u
            self.fid.write("%f,%f,%f,%f,%f,%f,%f,%f\n" % (time.time(),t,x,r,self.pidstate[0],self.pidstate[1],self.pidstate[2],uc) )
        # return u
        return uc
