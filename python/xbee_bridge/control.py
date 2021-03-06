## @file
# Control object for controlling EMILY

## TODO add logic to reset PID object when the control mode switch from teleoperation to automatic control

# import the definition of the gps class object
from xbee_bridge_state import gps_state
import math
# we plan on reading in numpy vectors - do I need to import numpy for this??
import numpy as np

headingThresholdDegrees = 40.0
headingThreshold = headingThresholdDegrees*math.pi/180.0
## range threshold: if closer than this value we assume we're at the target
rangeThreshold = 5.0

## Compute the heading error h1 - h2
def headingError(h1,h2):
    while h1 - h2 >= math.pi:
        h1 = h1 - 2.0*math.pi
    while h1 - h2 <= -math.pi:
        h1 = h1 + 2.0*math.pi
    return h1-h2

## Control class called from main loop
#
# @param[in] logDir relative file path to where logs are. If None, does not write logs. Else, PID object writes a diagonostic log with object information.
# @param[in] Kp proportional gains on heading during cruise
# @param[in] Ki integral gain on heading during cruise
# @param[in] Kd derivative gain on heading during cruise
# @param[in] turnThrottle: the throttle value to use in "turning" mode, on [0.0,1.0]
# @param[in] cruiseThrottle: the throttle to use while driving toward a waypoint, on [0.0 1.0]
class control():
    def __init__(self,logDir=None,Kp=0.0,Kd=0.0,Ki=0.0,turnThrottle=0.3,cruiseThrottle=0.6):
        # allocate memory
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.psi = 0.0
        self.rangeRef = 50.0
        ## The throttle setting while turning
        self.throttleWhileTurning = turnThrottle
        ## Throttle setting while doing PID on heading
        self.throttleWhileCruising = cruiseThrottle
        ## Target heading (rads) (inertial frame)
        self.headingRef = 0.0
        ## target rudder setting
        self.rudder = 0.0
        ## target throttle setting
        self.throttle = 0.0
        ## time for the fixed-action pattern for determining heading
        self.headingTime=0.0
        ## booleam associated with the heading fixed action pattern
        self.headingMoveBool=False
        ## State at the start of the heading fixed-action pattern
        self.headingMoveStateStart=np.zeros(4)
        ## State at the start of the heading fixed-action pattern
        self.headingMoveStateEnd=np.zeros(4)
        ## PID object for heading commands
        self.headingPid = PIDclass(Kp,Ki,Kd,logDir=logDir,name='hdg',umin=-1.0,umax=1.0)
        ## log object
        if logDir is not None:
            self.log = open(logDir+'controlObjectLog.csv','w+')
            self.log.write('time(sec),x(m),y(m),v(m/s),hdg(rads),rangeRef(m),headingRef(rad),rudder,throttle\n')
        else:
            self.log = None
    ## Convert a commanded vector (2-length array) to range and heading and store
    #
    # @param[in] mag the magnitude of the desired direction
    # @param[in] direc the direction in radians, relative to EMILY! Positive is to the right of EMILY (facing toward the front of boat)
    def vectorRef(self,mag,direc):
        self.rangeRef = mag
        self.headingRef = direc+self.psi
        return
    ## Convert an input state of x, y, v, and hdg
    #
    # @param[in] inState: a numpy array of input: [x(m),y(m),V(m/s),heading(radians)]
    def updateState(self,inState):
        self.x = inState[0]
        self.y = inState[1]
        self.v = inState[2]
        self.psi = inState[3]
        return
    ## Update the control value based on the current state
    # @param[in] tNow the current clock time (seconds)
    # @param[out] Reuturn 0 nominally. Return 1 if we are the waypoint. Return -1 if we just finished the fixed-action pattern.
    def update(self,tNow):
        # if in headingMove mode, perform that action
        if self.headingMoveBool>0:
            self.headingMoveAction(tNow)
            # log to file
            self.logSelf(tNow)
            if self.headingMoveBool==0:
                return -1
            return 0
        # evaluate range to target. Do nothing if close enough
        if self.rangeRef < rangeThreshold:
            self.rudder = 0.0
            self.throttle = 0.0
            # log to file
            self.logSelf(tNow)
            return 1
        # else, we're far from target: set the throttle on so we can figure out our speed
        self.throttle = self.throttleWhileTurning

        deltaPsi = headingError(self.psi,self.headingRef)
        if deltaPsi > headingThreshold:# we are more than 20 degrees east of the target
            # turn left at a slow speed
            #self.throttle = self.throttleWhileTurning
            self.rudder = -1.0
            print("%13s,%8.6f,%8.6f,%6.4f,%6.4f" % ("TURNING MODE",self.headingRef, self.rangeRef, self.rudder, self.throttle))
        elif deltaPsi < -headingThreshold: # we are more than 20 degrees west of the target
            # turn right at a slow speed
            #self.throttle = self.throttleWhileTurning
            self.rudder = 1.0
            print("%13s,%8.6f,%8.6f,%6.4f,%6.4f" % ("TURNING MODE",self.headingRef, self.rangeRef, self.rudder, self.throttle))
        else:# TODO do PID on heading
            self.rudder = self.headingPid.update(tNow,self.psi,self.headingRef,diffFun=headingError)
            self.throttle = self.throttleWhileCruising
            print("%13s,%8.6f,%8.6f,%6.4f,%6.4f" % ("PID MODE",self.headingRef, self.rangeRef, self.rudder, self.throttle))
        # log to file
        self.logSelf(tNow)
        return 0
    ## Write current state to file
    #
    # @param[in] tNow the current time
    def logSelf(self,tNow):
        #self.log.write('time(sec),x(m),y(m),v(m/s),hdg(rads),rangeRef(m),headingRef(rad),rudder,throttle\n')
        if self.log is not None:
            self.log.write('%.12g,%g,%g,%g,%g,%g,%g,%g,%g\n' % (tNow,self.x,self.y,self.v,self.psi,self.rangeRef,self.headingRef,self.rudder,self.throttle))
    ## Reset the PID object
    def reset(self):
        self.headingPid.reset()
    ## Called by main loop to enable the fixed-action pattern to determine heading
    #
    #@param[in] tNow current system time
    #@param[in] boolIn Boolean value
    def headingMove(self,tNow,boolIn):
        self.headingMoveBool=boolIn
        if boolIn>0:
            # record the current position
            self.headingMoveStateStart[0] = self.x
            self.headingMoveStateStart[1] = self.y
            self.headingMoveStateStart[2] = self.v
            self.headingMoveStateStart[3] = self.psi
            # set the initial time
            print("Started fixed-action pattern")
            self.headingTime=tNow
    ## Perform the fixed-action pattern to get the heading
    def headingMoveAction(self,tNow):
        # throttle on
        self.throttle=self.throttleWhileCruising
        # rudder zero
        self.rudder=0.0
        # timeout is 5 Secs
        if tNow - self.headingTime >= 5.0:
            # record the current position
            self.headingMoveStateEnd[0] = self.x
            self.headingMoveStateEnd[1] = self.y
            self.headingMoveStateEnd[2] = self.v
            self.headingMoveStateEnd[3] = self.psi
            # unset the flag
            self.headingMoveBool=False
            print("End fixed-action pattern")


## Function used to compute the error between state and reference in the default case
# subtracts the reference from the state
def defaultDifference(x,r):
    return x-r

## PID class from old Crazyflie code. For purely SISO systems.
class PIDclass:
    def __init__(self,Kp,Ki,Kd,logDir = None,name = 'pid1',umax = 1e9,umin = -1e9):
        ## last value of state, X
        self.stateLast = 0.0
        ## last timestamp
        self.timeLast = 0.0
        ## current P, I, D state values
        self.pidstate = np.array([0.0,0.0,0.0])
        ## PID gain array
        self.pidarray = np.array([Kp,Ki,Kd])
        ## store control copy locally
        self.u = np.array([0.0,0.0,0.0])
        ## flag that says if we've received data before - don't use integral term until this has been set
        # otherwise we get a huge jump in delta-time and integral term explodes
        self.flag_init = False
        ## control limits (use arbitrary large numbers if no relevant limits)
        self.u_max = umax
        self.u_min = umin
        # log file
        if logDir is not None:
            #open log file
            self.fid = open(logDir + ('pid_%s.csv' % (name)),'w')
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
        self.stateLast = 0.0
        self.u = np.array([0.0,0.0,0.0])
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
            inte = (t-self.timeLast)*prop
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
        # save timestamp
        self.timeLast = t
        # save last state
        self.stateLast = prop
        # save control locally
        self.u = uc
        # log control
        if self.fid is not None:
            # write time, x, r, prop, inte, deri, u
            self.fid.write("%f,%f,%f,%f,%f,%f,%f\n" % (t,x,r,self.pidstate[0],self.pidstate[1],self.pidstate[2],uc) )
        # return u
        return uc
