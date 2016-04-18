''' Test serial communication using the C extension '''
import sys
sys.path.append("build/lib.linux-i686-2.7")

import eps
import time
import serial
import struct

def getfield(strin):
    splt = strin.split(':')
    field = splt[0]
    try:
        val = float(splt[1])
    except ValueError:
        val = 0
        field = ''
    return (field,val)

# global log files
gpslog = open('gps.csv','w')
gpslog.write('systime(sec),time(sec),lat(deg),lon(deg),v(m/s),hdg(rad),x(m),y(m),newcmd(flag)\n')
ctrllog = open('ctrl.csv','w')
ctrllog.write('systime(sec),rudder(pwm),throttle(pwm),mode(flag),status(flag)\n')
gpscmd = open('gpscmd.csv','w')
gpscmd.write('systime(sec),lat(deg),lon(deg),x(m),y(m)\n')
## Parse the ASCII debug text from the Arduino
def parse_debug(chBuffer,systime):
    lines = chBuffer.split('\n')
    for k in lines:
        if len(k) < 4:
            continue
        else:
            # space delimit
            spl = k.split(' ')
            if spl[0] == 'GPS':
                outs = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
                for j in range(1,len(spl)):
                    (field,val) = getfield(spl[j])
                    if field == 'Time':
                        outs[0] = val
                    if field == 'Lat':
                        outs[1] = val*1.0e-7
                    if field == 'Lon':
                        outs[2] = val*1.0e-7
                    if field == 'V':
                        outs[3] = val
                    if field == 'H':
                        outs[4] = val
                    if field == 'X':
                        outs[5] = val
                    if field == 'Y':
                        outs[6] = val
                    if field == 'NEWCMD':
                        outs[7] = val
                    #print("%s:%f" % (field,val))
                gpslog.write("%16.4f,%10g,%.10g,%.10g,%g,%g,%g,%g,%g\n" % (systime,outs[0],outs[1],outs[2],outs[3],outs[4],outs[5],outs[6],outs[7]))
            if spl[0] == 'CTRL':
                outs = [0.0,0.0,0.0,0.0]
                for j in range(1,len(spl)):
                    (field,val) = getfield(spl[j])
                    #print("%s:%f" % (field,val))
                    if field == 'R':
                        outs[0] = val
                    if field == 'T':
                        outs[1] = val
                    if field == 'M':
                        outs[2] = val
                    if field == 'S':
                        outs[3] = val
                ctrllog.write("%16.4f,%g,%g,%d,%d\n" % (systime,outs[0],outs[1],int(outs[2]),int(outs[3])))
            if spl[0] == 'GPSCMD':
                outs = [0.0,0.0,0.0,0.0]
                for j in range(1,len(spl)):
                    (field,val) = getfield(spl[j])
                    #print("%s:%f" % (field,val))
                    if field == 'Lat':
                        outs[0] = val*1.0e-7
                    if field == 'Lon':
                        outs[1] = val*1.0e-7
                    if field == 'X':
                        outs[2] = val
                    if field == 'Y':
                        outs[3] = val
                gpscmd.write("%16.4f,%10g,%10g,%d,%d\n" % (systime,outs[0],outs[1],outs[2],outs[3]))


SERIAL_RATE_HZ = 2.0
SERIAL_READ_RATE_HZ = 10.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
SERIAL_READ_PERIOD = 1.0/SERIAL_READ_RATE_HZ
PORT = '/dev/ttyUSB0'

DEBUG_PORT = '/dev/ttyACM0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

serDebug = serial.Serial(DEBUG_PORT,9600,timeout=0.1)

ser.open()

counter = 0
tnext = time.clock() + SERIAL_PERIOD
tnext2 = time.clock() + SERIAL_READ_PERIOD
ch = ''
chDebug = ''
try:
    while True:#counter < 20:
        if time.clock() >= tnext2:
            parse_debug(chDebug,time.time())
            chDebug = ''
            tnext2 = tnext2 + SERIAL_READ_PERIOD
        if time.clock() >= tnext:
            #print(chDebug)
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
            (msg,lenv) = eps.pack_gps(-963400000,306200000,float(tnext))
            if lenv > 0:
                ser.write(msg)
                #print("Sent test GPS message:")
                #print(msg.encode("hex"))
            (msg,lenv) = eps.pack_control(0.0,0.0)
            if lenv > 0:
                ser.write(msg)
                #print("Sent test control message")
        # echo anything we receive
        while ser.inWaiting() > 0:
            ch += ser.read()
        while serDebug.inWaiting() > 0:
            chDebug += serDebug.read()
except KeyboardInterrupt:
    ser.close()
    # close logs
    gpslog.close()
    ctrllog.close()
    gpscmd.close()
