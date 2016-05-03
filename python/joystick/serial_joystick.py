''' Test serial communication using the C extension '''

## @file
import sys
sys.path.append('../ESP')
import esp_python as esp
import time
import serial
import struct

import pygame
pygame.init()

import sys

## Function for parsing lines from the serial debugging output
def getfield(strin):
    splt = strin.split(':')
    field = splt[0]
    try:
        val = float(splt[1])
    except ValueError:
        val = 0
        field = ''
    return (field,val)

## serial port settings
SERIAL_RATE_HZ = 10.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
PORT = 'COM4'

# DEBUG_PORT = '/dev/ttyACM0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

counter = 0
tnext = time.time() + SERIAL_PERIOD
ch = ''
parser = esp.espParser()

rudderCmd = 0.0
throttleCmd = 0.0

## joystick settings
joysticks = []
clock = pygame.time.Clock()
keepPlaying = True

for i in range(0, pygame.joystick.get_count()):
	# create an Joystick object in our list
    joysticks.append(pygame.joystick.Joystick(i))
    # initialize them all (-1 means loop forever)
    joysticks[-1].init()
    # print a statement telling what the name of the controller is
    print "Detected joystick '",joysticks[-1].get_name(),"'"

gpsLog = open('gpslog.csv','w')
gpsLog.write('systime,t,lon,lat,v,hdg,status\n')
controlLog = open('controlLog.csv','w')
controlLog.write('systime,rudd,thro\n')
controlOutLog = open('controlOutLog.csv','w')
controlOutLog.write('systime,rudd,thro\n')

while keepPlaying:
    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            axis_value = event.value
            if event.axis==0:
                rudderCmd = axis_value
            if event.axis==3:
                throttleCmd = -axis_value
                if throttleCmd <= 0.1:
                    throttleCmd = 0.0
                elif throttleCmd >= 1.0:
                    throttleCmd = 1.0
            print("Rudder = %f, Throttle = %f" % (rudderCmd,throttleCmd))
    tNow = time.time()
    if tNow >= tnext:
        (num,msgs) = parser.parseBytes(ch)
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
        		controlLog.write('%.12g,%f,%f\n' % (tNow,rudd,thro))
        	if msg_id == esp.message_command():
        		(len2,hdg,spd) = esp.unpack_command(msg)
        		print("COMM COMMAND: %f,%f" % (hdg,spd))
        	if msg_id == esp.message_set_pid():
        		(len2,ch0,Kp,Ki,Kd) = esp.unpack_set_pid(msg)
        		print("COMM SET_PID: %i,%f,%f,%f" % (ch0,Kp,Ki,Kd))
        	if msg_id == esp.message_gps_pos():
        		(len2,lon,lat,t,v,hdg,status) = esp.unpack_gps_pos(msg)
        		print("COMM GPS: %d,%d,%f" % (lon,lat,t))
        		gpsLog.write('%.12g,%g,%i,%i,%g,%g,%d\n' % (tNow,t,lon,lat,v,hdg,status))
    	ch = ''
        (msg) = esp.pack_control(rudderCmd,throttleCmd)
        controlOutLog.write('%.12g,%f,%f\n' % (tNow,rudderCmd,throttleCmd))
    	if len(msg) > 0:
        	ser.write(msg)
        	#print("Sent test control message")
        # echo anything we receive
        while ser.inWaiting() > 0:
            ch += ser.read()
        tnext = tnext + SERIAL_PERIOD
#except KeyboardInterrupt:
#    ser.close()
