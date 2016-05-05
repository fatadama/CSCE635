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
# import os for making directory
import os
# hardware interface class
import hardware_interface
# emily serial protocol
import esp_python as esp
# import the bridge class definition
from xbee_bridge_state import xbee_bridge_state
# import the joystick class
import joystick

## Class object for the XBee bridge
#
# @param[in] logDir: the relative path to create log files. Use empty string to use the current directory.
class bridgeProcess():
    def __init__(self,logDir = ''):
        ## state object from xbee_bridge_state
        self.state = xbee_bridge_state()
        ## start time - used to reduce all times to something more convenient for floats
        self.tStart = time.time()
        ## time at which to trigger 1Hz loop
        self.time_1Hz = 0.0
        ## time at which to trigger 10 Hz loop
        self.time_10Hz = 0.0
        ## time at which to trigger 50 Hz loop
        self.time_50Hz = 0.0
        ## joystick interface class
        self.joy = joystick.joystick()
        ## hardware_interface class object with default SIL arguments
        self.xbee = hardware_interface.hardware_interface(port=None,SIL=True,groundstation=True)
        self.xbee.start()
        ## parser object for the serial protocol
        self.espParser = esp.espParser()
        ## (boolean) set to True when we get a new GPS message
        self.new_data = False
        ## buffer of bytes to write to XBee; clear whenever written
        self.txBuffer = bytes()
        ## control mode: 0 == teleoperation, 1 == automatic control based on desired vector
        self.control_mode = 0
        # make log folder based on date/time
        dt = datetime.now()
        foldername = logDir+('%04d%02d%02d_%02d%02d%02d' % (dt.year,dt.month,dt.day,dt.hour,dt.minute,dt.second))
        if not os.path.exists(os.path.dirname(foldername)):
            os.makedirs(foldername)
        ## GPS log file
        self.gpsLog = open(foldername+'/gpslog.csv','w')
        self.gpsLog.write('systime,t,lon,lat,v,hdg,status\n')
        ## Control (received) log file
        self.controlLog = open(foldername+'/controlLog.csv','w')
        self.controlLog.write('systime,rudd,thro\n')
        ## Control (transmitted) log file
        self.controlOutLog = open(foldername+'/controlOutLog.csv','w')
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
        return
    def loop_10Hz(self,tNow):
        if control_mode == 0:# if control_mode == teleop, pass through joystick
            self.joy.read()
            # put rudder, throttle in Xbee TX buffer
            self.txBuffer+=esp.pack_control(self.joy.rudderCmd,self.joy.throttleCmd)
        elif control_mode == 1:# if control_mode == pfields, compute control
            # TODO add logic for converting an input vector to range/heading, and add control function for doing control
            self.txBuffer+=esp.pack_control(0.0,0.0)
        return
    def loop_50Hz(self,tNow):
        # Read XBee
        ch = self.xbee.read()
        # parse from xbee
        (num,msgs) = self.espParser.parseBytes(ch)
        num = len(msgs)
        # parse messages
        for k in range(num):
            # message id
            self.handle_msg(msgs[k],tNow)
        # Write buffer to XBee
        if len(self.txBuffer) > 0:
            self.xbee.write(self.txBuffer)
            # clear the TX buffer
            self.txBuffer = bytes()
        # Read IPC
        #   Update control_mode
        #   If control_mode == pfields
        #       Update target vector
        # If new_data
        if self.new_data:
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
            (len2,lon,lat,t,v,hdg,status) = esp.unpack_gps_pos(msg)
            # set new_data to true
            self.new_data = True
            # TODO filter state
            # write to state
            self.state.gpsState.update(lon,lat,t,v,hdg)
            # log to file
            self.log_gps(tNow,lon,lat,t,v,hdg)
            print("GPS_POS: %d,%d,%f,%f,%f,%d" % (lon,lat,t,v,hdg,status))
        if msg_id == esp.message_control():
            (len2,rudd,thro) = esp.unpack_control(msg)
            # log to file
            self.log_control(tNow,rudd,thro)
            print("CONTROL: %f,%f,t=%f" % (rudd,thro,tNow))
        if msg_id == esp.message_heartbeat():
            (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
            # do stuff
            print("HEARTBEAT: %i,%i,%f,t=%f" % (source_id,dest_id,syst,tNow))
    ## function to log GPS
    #
    # @param[in] tNow the current system time
    # @param[in] lon the longitude (positive north) from parsed message, units are (10^-7 deg). Convert to degrees by multiplying by 10^-7
    # @param[in] lat the latitude (positive east) from parsed message
    # @param[in] t the time from parsed message
    # @param[in] v the speed from parsed message (m/s)
    # @param[in] hdg the heading from parsed message (radians)
    def log_gps(self,tNow,lon,lat,t,v,hdg):
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


def main():
    process = bridgeProcess()
    try:
        while True: # main loop
            process.main_loop()
    except KeyboardInterrupt:
        print("Leaving xbee_bridge")
        # shutdown function calls
    return 0

if __name__ == "__main__":
    main()
