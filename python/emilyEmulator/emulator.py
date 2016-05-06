# add path to nanomsg
import sys
sys.path.append('../../../nanomsg-python')
# path to the GPS state class in the bridge object
sys.path.append('../xbee_bridge')
# path to ESP
sys.path.append('../ESP')

# gps_state class from xbee_bridge_state
from xbee_bridge_state import gps_state,GPS_D2R_RE
# math for pi
import math
# time for timing operations
import time
# ESP: Emily Serial protocol
import esp_python as esp
# hardware interface object for talking to the ''Xbee''
import hardware_interface
# numpy for arrays
import numpy as np
# scipy integration
import scipy.integrate
# struct for byte parssing
import struct

# initial GPS state
# Lake Bryan: 30 deg 42'37.2"N 96 deg 28'06.2"W
# max length of xbee buffer size
XBEE_BUFFER_SIZE_MAX = 256

## Dynamics for emily
# x[0] = x position
# x[1] = y position
# x[2] = speed
# x[3] = heading
# x[4] = rudder
# x[5] = throttle
# u[0] = rudder COMMAND on [-1,1]
# u[1] = throttle COMMAND on [0,1]
def eqom(x,t,u):
    dx = np.zeros(6)
    # enforce no zero velocity
    if x[2] < 0.0:
        x[2] = 0.0
    dx[0]=x[2]*math.cos(x[3])
    dx[1]=x[2]*math.sin(x[3])
    dx[2]=1.030544*x[5]-.271589
    dx[3]=0.16412*x[4] # ignore the bias for now
    #dx[3]=0.16412*x[4]-.07814
    dx[4]=10.0*(u[0]-x[4])
    dx[5]=2.5*(u[1]-x[5])
    # enforce limits:
    if x[2] > 12.0:# max speed 12 m/s
        dx[2] = 0.0
    return dx

## Gps home position - purely for convenience
GPS_HOME_LAT = 30.7103333
GPS_HOME_LON = -93.4683888
## GPS standard error (meters)
SIGMA_GPS = 1.0
## GPS velocity error (m/s)
SIGMA_GPS_V = 0.1
## GPS Heading error (rads)
SIGMA_GPS_H = 0.5

class emilyEmulator:
    def __init__(self):
        ## TRUTH state
        self.truthState = gps_state()
        ## true velocity (m/s) and heading (rads)
        self.velstate = np.zeros(2)
        ## true actuator states: rudder, throttle
        self.actuators = np.zeros(2)
        ## measured/estimated state
        self.gpsState = gps_state()
        ## serial port parser object
        self.serialParser = esp.espParser()
        # initialize TRUE state: lon (EAST), lat (UP), time, speed, heading (rads)
        self.truthState.update(GPS_HOME_LON*1.0e7,GPS_HOME_LAT*1.0e7,0.0,0.0,-math.pi*0.25)
        # initialize measured state
        self.gpsState.update(GPS_HOME_LON*1.0e7,GPS_HOME_LAT*1.0e7,0.0,0.0,-math.pi*0.25)
        ## rudder setting
        self.rudd = 0.0
        # throttle setting
        self.thro = 0.0
        return
    ## return a packed array of bytes to send a CONTROL message of the current commands
    def writeControl(self):
        msg = esp.pack_control(self.rudd,self.thro)
        return msg
    ## return a packed array of bytes to send a GPS_POS message of the current state
    def writeGPS(self):
        # fail to send with probability 7%, based on data
        pf = np.random.uniform(0.0,1.0)
        if pf <= 0.93:
            # send message
            msg = esp.pack_gps_pos(int(1.0e7*self.gpsState.lon),int(1.0e7*self.gpsState.lat),self.gpsState.time,self.gpsState.v,self.gpsState.hdg,1)
        else:
            # garbage
            msg ='*'
        return msg
    ## return a packed heartbeat message
    def writeHeartbeat(self):
        msg = esp.pack_heartbeat(esp.ESP_ID_BOAT,esp.ESP_ID_GROUNDSTATION,self.gpsState.time)
        return msg
    ## Handle a new byte from the input buffer
    #
    # @param[in] a string of length 1 or greater received from the UDP
    def readCh(self,ch):
        (num,msgs) = self.serialParser.parseBytes(ch)
        num = len(msgs)
        for k in range(num):
            # message id
            msg_id = struct.unpack('B',msgs[k][2])[0]
            msg = msgs[k]
            if msg_id == esp.message_control():
                (len2,rudd,thro) = esp.unpack_control(msg)
                self.rudd = rudd
                self.thro = thro
                #print("RECV CONTROL: %f,%f" % (self.rudd,self.thro))
            if msg_id == esp.message_heartbeat():
                (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
                print("RECV HEARTBEAT: %i,%i,%f" % (source_id,dest_id,syst))
        return
    def sampleGps(self):
        # sample the current truth state to produce a new GPS state
        self.gpsState.lon = self.truthState.lon
        self.gpsState.lat = self.truthState.lat
        self.gpsState.time = self.truthState.time
        self.gpsState.x = self.truthState.x
        self.gpsState.y = self.truthState.y
        #self.gpsState.latLon2XY()
        # add error to the X-Y state
        self.gpsState.x=self.gpsState.x+np.random.normal(scale=SIGMA_GPS)
        self.gpsState.y=self.gpsState.y+np.random.normal(scale=SIGMA_GPS)
        # convert back to lat/lon
        self.gpsState.XY2latLon()
        # sample velocity and heading
        self.gpsState.v = self.velstate[0]+np.random.normal(scale=SIGMA_GPS_V)
        self.gpsState.hdg = self.velstate[1]+np.random.normal(scale=SIGMA_GPS_H)
    ## propagate the state based on the assumed dynamics
    def propagate(self,dt):
        # compute the state
        x0 = np.zeros(6)
        x0[0] = self.truthState.x
        x0[1] = self.truthState.y
        x0[2:4] = self.velstate.copy()
        x0[4:6] = self.actuators.copy()
        # propagate for dt
        y = scipy.integrate.odeint(eqom,x0,[0.0,dt],args=(np.array([self.rudd,self.thro]),) )
        #print(eqom(y[-1,:].transpose(),0.0,np.array([self.rudd,self.thro] )) )
        # update the state
        self.velstate = y[-1,2:4]
        self.actuators = y[-1,4:6]
        self.truthState.x = y[-1,0]
        self.truthState.y = y[-1,1]
        self.truthState.v = y[-1,2]
        self.truthState.hdg = y[-1,3]
        # compute the lat lon
        self.truthState.XY2latLon()
        # set the time
        self.truthState.time = self.truthState.time+dt

class process:
    def __init__(self):
        ## emily emulator object
        self.emily = emilyEmulator()
        ## clock time associated with 1 Hz loop
        self.timer_1Hz = time.time()
        ## clock time associated with 5 Hz loop
        self.timer_5Hz = time.time()
        ## clock time associated with 50 Hz loop
        self.timer_50Hz = time.time()
        ## xbee object for emulation
        self.xbee = hardware_interface.hardware_interface(port=None,SIL=True,groundstation=False)
        self.xbee.start()
        ## parser object for ESP
        self.parser = esp.espParser()
    def main_loop(self):
        tNow = time.time()
        if tNow >= self.timer_1Hz:
            self.loop_1Hz(tNow)
        if tNow >= self.timer_5Hz:
            self.loop_5Hz(tNow)
        if tNow >= self.timer_50Hz:
            self.loop_50Hz(tNow)
    def loop_1Hz(self,tNow):
        # get heartbeat message
        msg = self.emily.writeHeartbeat()
        # send heartbeat
        self.writeBuffer(msg)
        # reset the timer
        self.timer_1Hz = self.timer_1Hz + 1.0
        return
    def loop_5Hz(self,tNow):
        # HACK print the truth state
        #print("%g,%g,%g,%g" % (tNow,self.emily.velstate[0],self.emily.actuators[1],self.emily.thro))
        # get GPS and send to buffer
        self.emily.sampleGps()
        buf = bytes()
        # put GPS in send buffer
        buf+=self.emily.writeGPS()
        # get control and send to buffer
        buf+=self.emily.writeControl()
        # write the buffer
        self.writeBuffer(buf)
        # update the timer and return
        self.timer_5Hz = self.timer_5Hz + 0.2
        return
    def loop_50Hz(self,tNow):
        # read from buffer
        ch = self.xbee.read()
        self.emily.readCh(ch)
        # simulate the dynamics
        self.emily.propagate(0.02)
        # reset the timer
        self.timer_50Hz = self.timer_50Hz + 0.02
        return
    def writeBuffer(self,buf):
        self.xbee.write(buf)
        return

def main():
    proc = process()
    print("Emily emulator started")
    try:
        while True:
            proc.main_loop()
    except KeyboardInterrupt:
        print("Exiting emulator")
    return

if __name__ == '__main__':
    main()
