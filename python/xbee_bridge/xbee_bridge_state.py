# numpy for arrays and potentially for operations later
import numpy as np
# hardware interface class
import hardware_interface

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
        if self.ready:
            self.lon = lowpass(self.lon, 1.0e-7*float(lon_int), SMOOTH_ALPHA)
            self.lat = lowpass(self.lat, 1.0e-7*float(lat_int), SMOOTH_ALPHA)
            self.v = lowpass(self.v,vel,SMOOTH_ALPHA)
            self.hdg = lowpass(self.hdg,h,SMOOTH_ALPHA)
        else:
            self.lon = (1.0e-7*float(lon_int))
            self.lat = (1.0e-7*float(lat_int))
            self.v = vel
            self.hdg = h
            self.lon_home = self.lon
            self.lat_home = self.lat
            self.ready = True
        self.time = t
        # update the X Y values
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
    def __init__(self):
        ## gpsState object for holding the state of EMILY
        self.gpsState = gps_state();
        ## control mode: 0 == teleoperation, 1 == pfields
        self.control_mode = 0
        return
