''' Test serial communication using the C extension '''

import esp
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
SERIAL_READ_RATE_HZ = 5.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
SERIAL_READ_PERIOD = 1.0/SERIAL_READ_RATE_HZ
PORT = '/dev/ttyUSB0'

DEBUG_PORT = '/dev/ttyACM0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

try:
    serDebug = serial.Serial(DEBUG_PORT,9600,timeout=0.1)
except serial.serialutil.SerialException:
    serDebug = None

ser.open()
if serDebug is not None:
    serDebug.open()

counter = 0
tnext = time.time() + SERIAL_PERIOD
tnext2 = time.time() + SERIAL_READ_PERIOD
ch = ''
chDebug = ''
parser = esp.espParser()
try:
    while True:
        if time.time() >= tnext2:
            parse_debug(chDebug,time.time())
            chDebug = ''
            tnext2 = tnext2 + SERIAL_READ_PERIOD
        if time.time() >= tnext:
            #print(ch)
            (num,msgs) = parser.parseBytes(ch)
            #if not(type(out) == int):
            num = len(msgs)
            for k in range(num):
                # message id
                msg_id = struct.unpack('B',msgs[k][2])[0]
                msg = msgs[k]
                if msg_id == esp.message_gps():
                    (len2,lon,lat,t) = esp.unpack_gps(msg)
                    print("COMM GPS: %d,%d,%f" % (lon,lat,t))
                if msg_id == esp.message_control():
                    (len2,rudd,thro) = esp.unpack_control(msg)
                    print("COMM CONTROL: %f,%f,t=%f" % (rudd,thro,time.time()))
                if msg_id == esp.message_command():
                    (len2,hdg,spd) = esp.unpack_command(msg)
                    print("COMM COMMAND: %f,%f" % (hdg,spd))
                if msg_id == esp.message_set_pid():
                    (len2,ch0,Kp,Ki,Kd) = esp.unpack_set_pid(msg)
                    print("COMM SET_PID: %i,%f,%f,%f" % (ch0,Kp,Ki,Kd))
            ch = ''
            tnext = tnext + SERIAL_PERIOD
            (msg) = esp.pack_control(0.0,0.0)
            if len(msg) > 0:
                ser.write(msg)
                #print("Sent test control message")
        # echo anything we receive
        while ser.inWaiting() > 0:
            ch += ser.read()
        if serDebug is not None:
            while serDebug.inWaiting() > 0:
                chDebug += serDebug.read()
except KeyboardInterrupt:
    ser.close()
    # close logs
    gpslog.close()
    ctrllog.close()
    gpscmd.close()
