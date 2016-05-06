# numpy for arrays and potentially for operations later
import numpy as np
# hardware interface class
import hardware_interface
# filter dynamics equations
import filter_dynamics
# sys to change the path
import sys
# math for sqrt
import math
# path to the EKF class definition
sys.path.append('../../../estimation/filters/python/ekf')
# EKF class definition
import ekf

## Conversion factor for (degrees->radians->arc length at mean Earth equatorial radius)
GPS_D2R_RE = 111318.845
## Smoothing factor to use on measured GPS values
SMOOTH_ALPHA = 0.75

## Lowpass filter an input to return a new, smoothed state
#
# Typical values of alpha are 0.05-0.15
# @param[in] xlast the prior value
# @param[in] xnew the new measurement
# @param[in] alpha: the smoothing factor
# @param[out] xout: xout = alpha*xnew + (1.0-alpha)*xlast
def lowpass(xlast,xnew,alpha):
    return alpha*xnew + (1.0-alpha)*xlast

## TODO add EKF

class gps_state():
    def __init__(self):
        # longitude (degrees) positive == east
        self.lon = 0.0
        # latitude (degrees) positive == north
        self.lat = 0.0
        ## GPS time in seconds (from receiver)
        self.time = 0.0
        self.x = 0.0
        self.y = 0.0
        self.v = 0.0
        self.hdg = 0.0
        self.lon_home = 0.0
        self.lat_home = 0.0
        ## initialization flag: set to False until we receive at least one GPS update
        self.ready = False
    ## Read in raw lat/lon/time from the XBee and update the inernal state
    #
    # Smooth all inputs with a 0.1 RC lowpass filter
    # @param[in] lon_int ( longitude (deg) x 10^7 ) as an integer
    # @param[in] lat_int ( Latitude (deg) x 10^7 ) as an integer
    # @param[in] t time in seconds
    # @param[in] vel speed in m/s
    # @param[in] h heading in radians, relative to north. (positive == east)
    def update(self,lon_int,lat_int,t,vel,h):
        self.time = t
        if self.ready:
            self.lon = lowpass(self.lon, 1.0e-7*float(lon_int), SMOOTH_ALPHA)
            self.lat = lowpass(self.lat, 1.0e-7*float(lat_int), SMOOTH_ALPHA)
            self.v = lowpass(self.v,vel,SMOOTH_ALPHA)
            self.hdg = lowpass(self.hdg,h,SMOOTH_ALPHA)
            # update the X Y values
            self.latLon2XY()
        else:
            self.lon = (1.0e-7*float(lon_int))
            self.lat = (1.0e-7*float(lat_int))
            self.v = vel
            self.hdg = h
            self.lon_home = self.lon
            self.lat_home = self.lat
            self.ready = True
            # compute the X-Y values
            self.latLon2XY()
        return
    ## update the x y position using the current lat/lon measurement
    #
    # Recall: North-east-down local frame.
    # X = (lat-lat_home)*GPS_D2R_RE
    # Y = (lon-lon_home)*GPS_D2R_RE
    def latLon2XY(self):
        self.x = (self.lat-self.lat_home)*GPS_D2R_RE
        self.y = (self.lon-self.lon_home)*GPS_D2R_RE
        return
    ## Compute lat and lon computed from the current x and y state
    def XY2latLon(self):
        self.lat = self.x/GPS_D2R_RE+self.lat_home
        self.lon = self.y/GPS_D2R_RE+self.lon_home


class xbee_bridge_state():
    ## Constructor
    #
    # @param[in] logDir: relative file path to the directory to log.
    def __init__(self,logDir=None):
        ## gpsState object for holding the state of EMILY
        self.gpsState = gps_state()
        ## local copy of the EKF state: [x(m),y(m),V(m/s),heading(radians)]
        self.filterState = np.zeros(4)
        # process noise matrix for the zero-jerk filter
        Qkin = np.diag([math.pow(filter_dynamics.sigma_jerk,2.0),math.pow(filter_dynamics.sigma_jerk,2.0)])
        ## EKF object
        self.EKF = ekf.ekf(6,0,filter_dynamics.propagate,filter_dynamics.propGradient,filter_dynamics.processMatrix,Qk=Qkin)
        ## control mode: 0 == teleoperation, 1 == pfields
        self.control_mode = 0
        ## log object
        if logDir is not None:
            self.log = open(logDir+"bridgeStateLog.csv",'w+')
            # write the headers
            self.log.write('time(sec),X(m),Y(m),VX(m/s),VY(m/s),AX(m/s^2),AY(m/s^2),Xm(m),Ym(m),Vm(m),Hdgm(rad)')
            for k in range(6):
                for j in range(6):
                    self.log.write(',P_%d%d' % (k+1,j+1))
            self.log.write('\n')
        else:
            self.log = None
        return
    ## Update the state with a new GPS measurement from EMILY. Do filtering.
    #
    # @param[in] lon_int ( longitude (deg) x 10^7 ) as an integer
    # @param[in] lat_int ( Latitude (deg) x 10^7 ) as an integer
    # @param[in] t time in seconds
    # @param[in] vel speed in m/s
    # @param[in] h heading in radians, relative to north. (positive == east)
    def update(self,lon_int,lat_int,t,vel,h):
        if self.gpsState.ready==False:
            self.gpsState.update(lon_int,lat_int,t,vel,h)
            # initialize the filter
            xk0 = np.array([self.gpsState.x,self.gpsState.y,0.0,0.0,0.0,0.0]) # initial state
            Pk0 = np.diag([math.pow(filter_dynamics.sigma_gps,2.0),math.pow(filter_dynamics.sigma_gps,2.0),1.0,1.0,1.0,1.0]) # initial covariance
            self.EKF.init_P(xk0,Pk0,t)
        else:
            # update the raw GPS object
            self.gpsState.update(lon_int,lat_int,t,vel,h)
            # Compute the propagation interval for the filter
            dt = t-self.EKF.t
            # propagate the filter to the current time
            self.EKF.propagateOde(dt)
            # call the filter
            self.EKF.update(t,np.array([self.gpsState.x,self.gpsState.y]),filter_dynamics.measurement,filter_dynamics.measurementGradient,filter_dynamics.Rkin)
        # copy the filter state to local
        self.filterState[0:2] = self.EKF.xhat[0:2].copy()
        self.filterState[2] = np.sqrt( np.sum(np.power(self.EKF.xhat[2:4],2.0)) )
        # If we're moving, use the velocity to approximate the heading; else, use the GPS heading
        if self.filterState[2] > 1.0:
            self.filterState[3] = np.arctan2( self.EKF.xhat[3],self.EKF.xhat[2] )
        else:
            self.filterState[3] = self.gpsState.hdg
        # log the status
        if self.log is not None:
            self.log.write('%f,%f,%f,%f,%f,%f,%f' % (t,self.EKF.xhat[0],self.EKF.xhat[1],self.EKF.xhat[2],self.EKF.xhat[3],self.EKF.xhat[4],self.EKF.xhat[5]))
            self.log.write(',%f,%f,%f,%f' % (self.gpsState.x,self.gpsState.y,self.gpsState.v,self.gpsState.hdg))
            for kk in range(6):
                for j in range(6):
                    self.log.write(',%f' % (self.EKF.Pk[kk,j]))
            self.log.write('\n')
