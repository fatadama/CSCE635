''' Test serial communication using the C extension '''
import sys
sys.path.append("build/lib.linux-i686-2.7")

import eps
import time
import serial
import struct

SERIAL_RATE_HZ = 2.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
PORT = '/dev/ttyUSB0'

DEBUG_PORT = '/dev/ttyACM0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

serDebug = serial.Serial(DEBUG_PORT,9600,timeout=0.1)

ser.open()

counter = 0
tnext = time.clock() + SERIAL_PERIOD
ch = ''
chDebug = ''
while counter < 20:
    if time.clock() >= tnext:
        #print(ch.encode('hex'))
        print(chDebug)
        chDebug = ''
        out = eps.parse_buffer(ch)
        if not(type(out) == int):
            for k in range(out[0]):
                j = 2*k+1 # 1, 3, 5
                # message id
                msg_id = int(out[j+1])
                msg = ch[out[j]:]
                if msg_id == eps.message_gps():
                    (lon,lat,t,len2) = eps.unpack_gps(msg)
                    print("COMM GPS: %d,%d,%f" % (lon,lat,t))
                if msg_id == eps.message_control():
                    (rudd,thro,len2) = eps.unpack_control(msg)
                    print("COMM CONTROL: %f,%f" % (rudd,thro))
                if msg_id == eps.message_command():
                    (hdg,spd,len2) = eps.unpack_command(msg)
                    print("COMM COMMAND: %f,%f" % (hdg,spd))
                if msg_id == eps.message_set_pid():
                    (ch,Kp,Ki,Kd,len2) = eps.unpack_set_pid(msg)
                    print("COMM SET_PID: %i,%f,%f,%f" % (ch,Kp,Ki,Kd))
        ch = ''
        tnext = tnext + SERIAL_PERIOD
        counter = counter+1
        (msg,lenv) = eps.pack_gps(1600000000,-960000000,float(tnext))
        if lenv > 0:
            ser.write(msg)
            #print("Sent test GPS message")
        (msg,lenv) = eps.pack_control(0.0,0.0)
        if lenv > 0:
            ser.write(msg)
            #print("Sent test control message")
    # echo anything we receive
    while ser.inWaiting() > 0:
        ch += ser.read()
    while serDebug.inWaiting() > 0:
        chDebug += serDebug.read()

ser.close()
