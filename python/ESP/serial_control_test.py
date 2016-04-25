''' Test serial communication using the C extension '''

## @file

import esp
import time
import serial
import struct

import threading
# queue for threading communication
import Queue

import sys
sys.path.append('../keyboard_thread')
# keyboard_thread has the function definitions used to read the keyboard input in a separate thread
import keyboard_thread

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

#keyboard queue object
keyboard_queue = Queue.Queue()
keyboard_lock = threading.Lock()
keyboard_lock.acquire()#lock used only to terminate thread
key_thread = threading.Thread(target=keyboard_thread.input,args=(keyboard_queue,keyboard_lock))

## serial port settings
SERIAL_RATE_HZ = 10.0
SERIAL_PERIOD = 1.0/SERIAL_RATE_HZ
HEARTBEAT_RATE_HZ = 1.0
HEARTBEAT_PERIOD = 1.0/HEARTBEAT_RATE_HZ
PORT = '/dev/ttyUSB0'

DEBUG_PORT = '/dev/ttyACM0'

ser = serial.Serial(PORT,9600,timeout=0.01)
print("Opening port" + PORT)

ser.open()

counter = 0
tNow = time.time()
tnext = tNow + SERIAL_PERIOD
theartbeat = tNow + HEARTBEAT_PERIOD
ch = ''
parser = esp.espParser()

rudderCmd = 0.0
throttleCmd = 0.0

# start keyboard thread
key_thread.start()
try:
    while True:
        #read queues
        # keyboard queue
        if keyboard_queue.empty() and not key_thread.is_alive():
            break;
        else:
            try:
                line = keyboard_queue.get(timeout=0.01)
                print("Keyboard input: %s" % (line))
                # handle change in rudder
                if line[0] == 'a':
                    rudderCmd = rudderCmd - 0.05
                if line[0] == 'd':
                    rudderCmd = rudderCmd + 0.05
                if rudderCmd > 1.0:
                    rudderCmd = 1.0
                elif rudderCmd < -1.0:
                    rudderCmd = -1.0
                #print("Rudder to %f" % (rudderCmd))
                if line[0] == 'w':
                    throttleCmd = throttleCmd + 0.10
                if line[0] == 's':
                    throttleCmd = throttleCmd - 0.10
                if throttleCmd > 1.0:
                    throttleCmd = 1.0
                elif throttleCmd < 0.0:
                    throttleCmd = 0.0
                if line[0] == 'x':
                    throttleCmd = 0.0
                    rudderCmd = 0.0
                #print("Throttle to %f" % (throttleCmd))
                print("R=%d,T=%d" % (rudderCmd,throttleCmd))
            except Queue.Empty:
                pass
        tNow = time.time()
        if tNow >= theartbeat:
            (msg) = esp.pack_heartbeat(esp.ESP_ID_GROUNDSTATION,esp.ESP_ID_BOAT,tNow)
            if len(msg) > 0:
                ser.write(msg)
            theartbeat = theartbeat + HEARTBEAT_PERIOD
        if tNow >= tnext:
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
                    print("COMM CONTROL: %f,%f,t=%f" % (rudd,thro,tNow))
                if msg_id == esp.message_command():
                    (len2,hdg,spd) = esp.unpack_command(msg)
                    print("COMM COMMAND: %f,%f" % (hdg,spd))
                if msg_id == esp.message_set_pid():
                    (len2,ch0,Kp,Ki,Kd) = esp.unpack_set_pid(msg)
                    print("COMM SET_PID: %i,%f,%f,%f" % (ch0,Kp,Ki,Kd))
                if msg_id == esp.message_heartbeat():
                    (len2,source_id,dest_id,syst) = esp.unpack_heartbeat(msg)
                    print("COMM HEARTBEAT: %i,%i,%f" % (source_id,dest_id,syst))
            ch = ''
            tnext = tnext + SERIAL_PERIOD
            (msg) = esp.pack_control(rudderCmd,throttleCmd)
            if len(msg) > 0:
                ser.write(msg)
                #print("Sent test control message")
        # echo anything we receive
        while ser.inWaiting() > 0:
            ch += ser.read()
except KeyboardInterrupt:
    ser.close()
    # release lock(s) to stop threads
    keyboard_lock.release()
