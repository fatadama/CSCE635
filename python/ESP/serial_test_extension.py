''' Test serial communication using the C extension '''
import sys
sys.path.append('../ESP')

import eps
import time
import serial
import struct

SERIAL_RATE_HZ = 1.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
PORT = '/dev/ttyUSB0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

ser.open()

counter = 0
tnext = time.clock() + SERIAL_PERIOD
ch = ''
while counter < 20:
    if time.clock() >= tnext:
        print(ch.encode('hex'))
        out = eps.parse_buffer(ch)
        for k in range(out[0]):
            j = 2*k+1 # 1, 3, 5
            # message id
            msg_id = int(out[j+1])
            msg = ch[out[j]:]
            if msg_id == eps.message_gps():
                (lon,lat,t,len2) = eps.unpack_gps(msg)
                print("Unpacked: %d,%d,%f" % (lon,lat,t))
            if msg_id == eps.message_control():
                (rudd,thro,len2) = eps.unpack_control(msg)
                print("Unpacked: %f,%f" % (rudd,thro))
            if msg_id == eps.message_command():
                (hdg,spd,len2) = eps.unpack_command(msg)
                print("Unpacked: %f,%f" % (hdg,spd))
            if msg_id == eps.message_set_pid():
                (ch,Kp,Ki,Kd,len2) = eps.unpack_set_pid(msg)
                print("Unpacked: %i,%f,%f,%f" % (ch,Kp,Ki,Kd))
        ch = ''
        tnext = tnext + SERIAL_PERIOD
        counter = counter+1
        (msg,lenv) = eps.pack_gps(1600000000,-960000000,float(tnext))
        #print("Checksum:",esp.checksum_valid(msg,16))
        '''for k in range(len(msg)):
            print(k,struct.unpack('B',msg[k]))'''
        ser.write(msg)
    # echo anything we receive
    while ser.inWaiting() > 0:
        ch += ser.read()

ser.close()
