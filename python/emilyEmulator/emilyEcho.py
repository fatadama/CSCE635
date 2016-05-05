## @file emilyEcho
# read from the xbee_bridge class and then write to screen

import sys
sys.path.append('../xbee_bridge')
sys.path.append('../ESP')

import hardware_interface
import esp_python as esp
import struct

parser = esp.espParser()

server = hardware_interface.hardware_interface(port=None,groundstation=False,SIL=True)
server.start()

while True:
    try:
        ch = server.read()
        # parse
        (num,msgs) = parser.parseBytes(ch)
        num = len(msgs)
        for k in range(num):
            # message id
            msg_id = struct.unpack('B',msgs[k][2])[0]
            msg = msgs[k]
            if msg_id == esp.message_gps_pos():
                (len2,lon,lat,t,v,hdg) = esp.unpack_gps_pos(msg)
                print("GPS_POS: %d,%d,%f,%f,%f" % (lon,lat,t,v,hdg))
            if msg_id == esp.message_control():
                (len2,rudd,thro) = esp.unpack_control(msg)
                print("CONTROL: %f,%f" % (rudd,thro))
            if msg_id == esp.message_command():
                (len2,hdg,spd) = esp.unpack_command(msg)
                print("COMMAND: %f,%f" % (hdg,spd))
            if msg_id == esp.message_set_pid():
                (len2,ch0,Kp,Ki,Kd) = esp.unpack_set_pid(msg)
                print("SET_PID: %i,%f,%f,%f" % (ch0,Kp,Ki,Kd))
            if msg_id == esp.message_heartbeat():
                (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
                print("HEARTBEAT: %i,%i,%f" % (source_id,dest_id,syst))
    except KeyboardInterrupt:
        print("exit emilyEcho")
        break;
