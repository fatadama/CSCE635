# add path to nanomsg
import sys
sys.path.append('../../../nanomsg-python')
# path to the GPS state class in the bridge object
sys.path.append('../xbee_bridge')
# path to ESP
sys.path.append('../ESP')

# gps_state class from xbee_bridge_state
from xbee_bridge_state import gps_state
# math for pi
import math
# time for timing operations
import time
# ESP: Emily Serial protocol
import esp_python as esp

# initial GPS state
# Lake Bryan: 30 deg 42'37.2"N 96 deg 28'06.2"W
# max length of xbee buffer size
XBEE_BUFFER_SIZE_MAX = 256

class emilyEmulator:
    def __init__(self):
        ## TRUTH state
        self.truthState = gps_state()
        ## measured/estimated state
        self.gpsState = gps_state()
        ## serial port parser object
        self.serialParser = esp.espParser()
        # initialize TRUE state: lon (EAST), lat (UP), time, speed, heading (rads)
        self.truthState.update(964683888,307103333,0.0,0.0,-math.pi*0.25)
        # initialize measured state
        self.gpsState.update(964683888,307103333,0.0,0.0,-math.pi*0.25)
        return
    ## return a packed array of bytes to send a GPS_POS message of the current state
    def writeGPS(self):
        msg = esp.pack_gps_pos(int(1.0e7*self.gpsState.lon),int(1.0e7*self.gpsState.lat),self.gpsState.time,self.gpsState.v,self.gpsState.hdg,1)
        return msg
    ## return a packed heartbeat message
    def writeHeartbeat(self):
        msg = esp.pack_heartbeat(esp.ESP_ID_BOAT,esp.ESP_ID_GROUNDSTATION,self.gpsState.time)
        return msg
    ## Handle a new byte from the input buffer
    #
    # @param[in] a string of length 1 or greater received from the IPC
    def readCh(self,ch):
        (num,msgs) = self.serialParser.parseBytes(ch)
        num = len(msgs)
        for k in range(num):
            # message id
            msg_id = struct.unpack('B',msgs[k][2])[0]
            msg = msgs[k]
            if msg_id == esp.message_control():
                (len2,rudd,thro) = esp.unpack_control(msg)
                print("RECV CONTROL: %f,%f,t=%f" % (rudd,thro,tNow))
            if msg_id == esp.message_heartbeat():
                (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
                print("RECV HEARTBEAT: %i,%i,%f" % (source_id,dest_id,syst))
        return


class process:
    def __init__(self):
        ## emily emulator object
        self.emily = emilyEmulator()
        ## clock time associated with 1 Hz loop
        self.timer_1Hz = time.time()
        ## clock time associated with 10 Hz loop
        self.timer_10Hz = time.time()
        ## clock time associated with 100 Hz loop
        self.timer_100Hz = time.time()
    def main_loop(self):
        tNow = time.time()
        if tNow >= self.timer_1Hz:
            self.loop_1Hz(tNow)
        if tNow >= self.timer_10Hz:
            self.loop_10Hz(tNow)
        if tNow >= self.timer_100Hz:
            self.loop_100Hz(tNow)
    def loop_1Hz(self,tNow):
        # get heartbeat message
        msg = self.emily.writeHeartbeat()
        # send heartbeat
        self.writeBuffer(msg)
        # reset the timer
        self.timer_1Hz = self.timer_1Hz + 1.0
        return
    def loop_10Hz(self,tNow):
        # get GPS and send to buffer
        buf = bytes()
        buf+=self.emily.writeGPS()
        # get control and send to buffer
        # put GPS in send buffer
        self.writeBuffer(buf)
        # update the timer and return
        self.timer_10Hz = self.timer_10Hz + 0.1
        return
    def loop_100Hz(self,tNow):
        # read the xbee buffer
        ch = ''
        # handle the buffer
        self.emily.readCh(ch)
        # update the timer and return
        self.timer_100Hz = self.timer_100Hz + 0.01
        return
    def writeBuffer(self,buf):
        print(buf)
        #print('Output buffer:'+buf)
        return

def main():
    proc = process()
    try:
        while True:
            proc.main_loop()
    except KeyboardInterrupt:
        print("Exiting emulator")
    return

if __name__ == '__main__':
    main()
