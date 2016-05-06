## @file
import sys
# add path to serial protocol
sys.path.append('../ESP')
# add path to nanomsg-python
sys.path.append('../../../nanomsg-python')
# add path to filter
sys.path.append('../../../estimation')

# serial for talking to xbee
import serial
# time for running at fixed rates
import time
# struct for handling bytes
import struct
# datetime to get the name of the log folder
from datetime import datetime
# import os for creating the log directory
import os
# import ConfigParser for loading ini file NOTE in Python 3 the name of this module changes
from ConfigParser import ConfigParser
# math for pi
import math

# hardware interface class
import hardware_interface
# emily serial protocol
import esp_python as esp
# import the bridge class definition
from xbee_bridge_state import xbee_bridge_state
# import the joystick class
import joystick
# import the control class
import control

## Class object for the XBee bridge
#
# @param[in] logDir: the relative path to create log files. Use empty string to use the current directory.
# @param[in] SIL: boolean, set to True to use the emily emulator, False to use XBee on specified port
# @param[in] port: string, the name of the serial port. Windows ports look like 'COMXX'. Not used if SIL is True
# @param[in] Kp: heading proportional gain
# @param[in] Ki: heading integral gain
# @param[in] Kd: heading derivative gain
# @param[in] dw_interface: boolean; set to True to enable SPACEBAR and BACKSPACE keys to send simple testing waypoints to EMILY
# @param[in] dw_radius: float, (meters) range at which synthetic waypoints are generated relative to current position. Used only if dw_interface is True.
# @param[in] dw_angle: float, angle (radians) at which synthetic waypoint is generated if dw_interface==True. Not used if dw_interface is False.
class bridgeProcess():
    def __init__(self,logDir = '',SIL=True,port='COM4',Kp=0.0,Kd=0.0,Ki=0.0,dw_interface=False,dw_radius=0.0,dw_angle=0.0):
        # make log folder based on date/time
        dt = datetime.now()
        foldername = logDir+('%04d%02d%02d_%02d%02d%02d/' % (dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second))
        if not os.path.exists(os.path.dirname(foldername)):
            os.makedirs(foldername)
            ## state object from xbee_bridge_state
        self.state = xbee_bridge_state(logDir=foldername)
        ## start time - used to reduce all times to something more convenient for floats
        self.tStart = time.time()
        ## time at which to trigger 1Hz loop
        self.time_1Hz = 0.0
        ## time at which to trigger 10 Hz loop
        self.time_10Hz = 0.0
        ## time at which to trigger 50 Hz loop
        self.time_50Hz = 0.0
        ## dw_interface: boolean, set to True to enable the debugging interface
        self.dw_interface = dw_interface
        ## dw_radius: range at which to create synthetic waypoints as float in meters
        self.dw_radius = dw_radius
        ## dw_angle: relative angle to synthetic waypoints as a float in radians, positive is to the right
        self.dw_angle = dw_angle
        ## synthetic waypoint object used in debugging
        self.syntheticWaypoint = synthetic_waypoint(logDir=foldername)
        ## joystick interface class
        self.joy = joystick.joystick(dw_interface)
        ## hardware_interface class object with default SIL arguments
        self.xbee = hardware_interface.hardware_interface(port=port,SIL=SIL,groundstation=True)
        self.xbee.start()
        ## (boolean) set to True when we get a new GPS message
        self.new_data = False
        ## buffer of bytes to write to XBee; clear whenever written
        self.txBuffer = bytes()
        ## control object for computing control
        self.control = control.control(logDir=foldername,Kp=Kp,Kd=Kd,Ki=Ki)
        ## parser object for the serial protocol. Pass the log folder to create message logs
        self.espParser = esp.espParser(logdir=foldername)
        ## GPS log file
        self.gpsLog = open(foldername+'gpslog.csv','w')
        self.gpsLog.write('systime,t,lon,lat,v,hdg,status\n')
        ## Control (received) log file
        self.controlLog = open(foldername+'controlLog.csv','w')
        self.controlLog.write('systime,rudd,thro\n')
        ## Control (transmitted) log file
        self.controlOutLog = open(foldername+'controlOutLog.csv','w')
        self.controlOutLog.write('systime,rudd,thro\n')
    def __del__(self):
        ## close the log files
        self.gpsLog.close()
        self.controlLog.close()
        self.controlOutLog.close()
    def main_loop(self):
        tNow = time.time()-self.tStart
        if (tNow >= self.time_1Hz): # 1 Hz
            self.time_1Hz = self.time_1Hz + 1.0
            self.loop_1Hz(tNow)
        if (tNow >= self.time_10Hz): # 10 Hz
            # execute the 10 Hz loop
            self.time_10Hz = self.time_10Hz + 0.1
            self.loop_10Hz(tNow)
        if (tNow >= self.time_50Hz): #50 Hz
            self.time_50Hz = self.time_50Hz + 0.02
            self.loop_50Hz(tNow)
    def loop_1Hz(self,tNow):
        # put XBee heartbeat in TX buffer
        self.txBuffer+=esp.pack_heartbeat(esp.ESP_ID_GROUNDSTATION,esp.ESP_ID_BOAT,tNow)
        # print the status
        print("Secs last message: %8.6f Secs heartbeat: %8.6f Secs GPS: %8.6f" % (tNow-self.state.timeLastMsg[0],tNow-self.state.timeLastMsg[1],tNow-self.state.timeLastMsg[2]))
        return
    def loop_10Hz(self,tNow):
        if self.state.control_mode == 0:# if control_mode == teleop, pass through joystick
            self.joy.read()
            # put rudder, throttle in Xbee TX buffer
            self.txBuffer+=esp.pack_control(self.joy.rudderCmd,self.joy.throttleCmd)
            self.log_controlOut(tNow,self.joy.rudderCmd,self.joy.throttleCmd)
        elif self.state.control_mode == 1:# if control_mode == pfields, compute control
            if self.dw_interface:# read the joystick to keep updating the inputs
                self.joy.read()
                # update the synthetic waypoint with the current state
                self.syntheticWaypoint.update(tNow,self.state.filterState[0],self.state.filterState[1],self.state.filterState[3])
                # update the reference to the control object
                self.control.vectorRef(self.syntheticWaypoint.range, self.syntheticWaypoint.bearing)
            # call the control object with the current state
            self.control.update(tNow)
            #print( self.control.headingRef, self.control.rangeRef, self.control.rudder, self.control.throttle )
            self.txBuffer+=esp.pack_control(self.control.rudder,self.control.throttle)
            self.log_controlOut(tNow,self.control.rudder,self.control.throttle)
        return
    def loop_50Hz(self,tNow):
        # Read XBee
        ch = self.xbee.read()
        # parse from xbee
        (num,msgs) = self.espParser.parseBytes(ch)
        num = len(msgs)
        # if get messages, save the time of last message
        if num > 0:
            self.state.timeLastMsg[0] = tNow
        # parse messages
        for k in range(num):
            # message id
            self.handle_msg(msgs[k],tNow)
        # Write buffer to XBee
        if len(self.txBuffer) > 0:
            self.xbee.write(self.txBuffer)
            # clear the TX buffer
            self.txBuffer = bytes()
        # check debugging interface to toggle settings
        if self.dw_interface:
            if self.joy.control_mode:
                if self.state.control_mode == 0:
                    # reset the PID object
                    self.control.reset()
                    # set the control_mode flag
                    self.state.control_mode = 1
                elif self.state.control_mode == 1:
                    self.state.control_mode = 0
                print("Toggle control mode")
                self.joy.control_mode = False
            if self.joy.new_waypoint:
                # TODO create new waypoint
                self.joy.new_waypoint = False
                self.syntheticWaypoint.create(self.dw_radius,self.dw_angle,self.state.filterState[0],self.state.filterState[1],self.state.filterState[3])
                print("Created waypoint at %g, %g" % (self.syntheticWaypoint.x,self.syntheticWaypoint.y))
        # Read IPC
        #   Update control_mode
        #   If control_mode == pfields
        #       Update target vector
        # If new_data
        if self.new_data:
            # update the state to the control object
            self.control.updateState(self.state.filterState)
            # TODO Write GPS to IPC
            # set flag to false
            self.new_data = False
        return
    ## Function to process ESP messages
    #
    # @param[in] msg the message bytes
    # @param[in] tNow, the current time, used for some operations
    def handle_msg(self,msg,tNow):
        # message id
        msg_id = struct.unpack('B',msg[2])[0]
        if msg_id == esp.message_gps_pos():
            # store the time
            self.state.timeLastMsg[2] = tNow
            # unpack
            (len2,lon,lat,t,v,hdg,status) = esp.unpack_gps_pos(msg)
            # set new_data to true
            self.new_data = True
            # TODO filter state
            # write to state
            self.state.update(lon,lat,t,v,hdg)
            # log to file
            self.log_gps(tNow,lon,lat,t,v,hdg,status)
            #print("GPS_POS: %d,%d,%f,%f,%f,%d" % (lon,lat,t,v,hdg,status))
        if msg_id == esp.message_control():
            (len2,rudd,thro) = esp.unpack_control(msg)
            # log to file
            self.log_control(tNow,rudd,thro)
            #print("CONTROL: %f,%f,t=%f" % (rudd,thro,tNow))
        if msg_id == esp.message_heartbeat():
            # save time
            self.state.timeLastMsg[1] = tNow
            # unpack
            (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
            # do stuff
            #print("HEARTBEAT: %i,%i,%f,t=%f" % (source_id,dest_id,syst,tNow))
    ## function to log GPS
    #
    # @param[in] tNow the current system time
    # @param[in] lon the longitude (positive north) from parsed message, units are (10^-7 deg). Convert to degrees by multiplying by 10^-7
    # @param[in] lat the latitude (positive east) from parsed message
    # @param[in] t the time from parsed message
    # @param[in] v the speed from parsed message (m/s)
    # @param[in] hdg the heading from parsed message (radians)
    # @param[in] status the status byte from message
    def log_gps(self,tNow,lon,lat,t,v,hdg,status):
        self.gpsLog.write('%.12g,%g,%i,%i,%g,%g,%d\n' % (tNow,t,lon,lat,v,hdg,status))
    ## Function to log received control values
    #
    # @param[in] tNow the current system time
    # @param[in] rudd the rudder command on [-1,1]
    # @param[in] thro the throttle command on [0,1]
    def log_control(self,tNow,rudd,thro):
        self.controlLog.write('%.12g,%g,%g\n' % (tNow,rudd,thro))
    ## Function to log transmitted control values
    #
    # @param[in] tNow the current system time
    # @param[in] rudd the rudder command on [-1,1]
    # @param[in] thro the throttle command on [0,1]
    def log_controlOut(self,tNow,rudd,thro):
        self.controlOutLog.write('%.12g,%g,%g\n' % (tNow,rudd,thro))

## Class for monitoring and updating the relative position of a synthetic waypoint for hardware testing of EMILY
#
# uses: call create(range, bearing, x, y, hdg) to create a waypoint at 'range' meters and 'bearing' radians relative to EMILY's 'x','y' location and heading 'hdg' in some interial reference frame. Heading is radians, positive = east of north.
#       call update(x,y,hdg) to update the values of a created waypoint based on EMILY's current 'x','y' lcoation and heading 'hdg' in radians.
#       values are stored in self.range, self.bearing
# @param[in] logDir filepath to place logs
class synthetic_waypoint(logDir=None):
    def __init__(self):
        ## Distance to target (meters)
        self.range = 0.0
        ## Bearing to target (radians)
        self.bearing = 0.0
        ## inertial x position of waypoint (meters)
        self.x = 0.0
        ## inertial y position of waypoint (meters)
        self.y = 0.0
        ## load log file
        if logDir is not None:
            self.log = open(logDir+'synWaypointLog.csv','w+')
            self.log.write('time(sec),x_wp(m),y_wp(m),range(m),bearing(rad)\n')
        else:
            self.log = None
    ## Create a synthetic waypoint at range and bearing relative to EMILY
    # @param[in] rho the range (meters)
    # @param[in] bearing the relative angle in radians(positive is to the right of EMILY)
    def create(self,rho, bearing, x, y, hdg):
        self.x = x+rho*math.cos(bearing+hdg)
        self.y = y+rho*math.sin(bearing+hdg)
    ## After a waypoint is created, compute the range and bearing to it
    def update(self,tNow,x,y,hdg):
        self.range = math.sqrt( math.pow(self.x-x,2.0)+math.pow(self.y-y,2.0) )
        self.bearing = math.atan2(self.y-y,self.x-x)-hdg
        print("waypoint: %g, %g" % (self.range,self.bearing))
        if self.log is not None:
            self.log.write('%.12g,%g,%g,%g,%g\n' % (tNow,self.x,self.y,self.range,self.bearing))

def main():
    # load settings
    settings = ConfigParser()
    settings.read('settings.ini')
    print('Loaded settings file settings.ini')
    # default values of settings
    SIL = False
    port = 'COM4'
    # heading control gains
    Kp = 0.0
    Ki = 0.0
    Kd = 0.0
    # Flag that enables SPACEBAR to toggle control mode and BACKSPACE sets a new fake waypoint near EMILY
    debug_pygame_interface = False
    # Distance from EMILY's current location at which to place a waypoint, in debugging mode
    debug_waypoint_radius = 5.0
    # Angle relative to EMILY (positive == right) in degrees at which to place the debug waypoint
    debug_waypoint_angle = 170.0
    for item in settings.items('xbee_bridge'):
        if item[0] == 'sil':
            if item[1]=='False':
                SIL = False
            if item[1]=='True':
                SIL = True
            print('Set SIL to %d' % (SIL))
        if item[0] == 'port':
            port = item[1]
            print('Set port to %s' % (port))
        if item[0] == 'kph':
            Kp = float(item[1])
            print('Heading proportional gain = %g' % Kp)
        if item[0] == 'kdh':
            Kd = float(item[1])
            print('Heading derivative gain = %g' % Kd)
        if item[0] == 'kih':
            Ki = float(item[1])
            print('Heading integral gain = %g' % Ki)
        if item[0] == 'debug_interface':
            if item[1]=='True':
                debug_pygame_interface=True
                print('Debug interface in pygame enabled')
                print("\tUse SPACE to toggle between teleoperation and automatic control mode ")
                print("\tUse BACKSPACE to generate a synthetic waypoint at debug_waypoint_radius meters and debug_waypoint_angle degrees")
        if item[0] == 'debug_waypoint_radius':
            debug_waypoint_radius=float(item[1])
            print('Fake waypoint target at %g meters' % (debug_waypoint_radius))
        if item[0] == 'debug_waypoint_angle':
            debug_waypoint_angle=float(item[1])
            print('Fake waypoint target at %g deg' % (debug_waypoint_angle))
    # convert target angle to radians
    debug_waypoint_angle = debug_waypoint_angle*math.pi/180.0
    process = bridgeProcess(SIL=SIL,port=port,Kp=Kp,Kd=Kd,Ki=Ki,dw_interface=debug_pygame_interface,dw_radius=debug_waypoint_radius,dw_angle=debug_waypoint_angle)
    try:
        while True: # main loop
            process.main_loop()
    except KeyboardInterrupt:
        print("Leaving xbee_bridge")
        # shutdown function calls
    return 0

if __name__ == "__main__":
    main()
