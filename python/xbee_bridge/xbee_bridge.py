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
# hardware interface class
import hardware_interface
# emily serial protocol
import esp_python as esp
# import the bridge class definition
from xbee_bridge_state import xbee_bridge_state

class bridgeProcess():
    def __init__(self):
        self.state = xbee_bridge_state()
        self.tStart = time.time()
        self.time_1Hz = 0.0
        self.time_10Hz = 0.0
        self.time_50Hz = 0.0
        ## hardware_interface class object with default SIL arguments
        self.xbee = hardware_interface.hardware_interface(port=None,SIL=True,groundstation=True)
        self.xbee.start()
        ## parser object for the serial protocol
        self.espParser = esp.espParser()
        ## flag: set to True when we get a new GPS message
        self.new_data = False
        ## xbee output buffer
        self.txBuffer = ''
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
        # if control_mode == teleop, pass through joystick
        # if control_mode == pfields, compute control
        # put rudder, throttle in Xbee TX buffer
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
            msg_id = struct.unpack('B',msgs[k][2])[0]
            msg = msgs[k]
            if msg_id == esp.message_gps_pos():
                (len2,lon,lat,t,v,hdg,status) = esp.unpack_gps_pos(msg)
                # set new_data to true
                self.new_data = True
                # TODO filter state
                # write to state
                self.state.gpsState.update(lon,lat,t,v,hdg)
                print("GPS_POS: %d,%d,%f,%f,%f,%d" % (lon,lat,t,v,hdg,status))
            if msg_id == esp.message_control():
                (len2,rudd,thro) = esp.unpack_control(msg)
                #do stuff
                print("CONTROL: %f,%f,t=%f" % (rudd,thro,tNow))
            if msg_id == esp.message_heartbeat():
                (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
                # do stuff
                print("HEARTBEAT: %i,%i,%f" % (source_id,dest_id,syst))
        # Write buffer to XBee
        if len(self.txBuffer) > 0:
            self.xbee.write(self.txBuffer)
            # clear the TX buffer
            self.txBuffer = ''
        # Read IPC
        #   Update control_mode
        #   If control_mode == pfields
        #       Update target vector
        # If new_data
        if self.new_data:
            #   Write GPS to IPC
            # set flag to false
            self.new_data = False

        return

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
