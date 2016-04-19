import esp
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
        ch = ''
        tnext = tnext + SERIAL_PERIOD
        counter = counter+1
        msg = esp.esp_pack_gps(3200000000,-960000000,float(tnext))
        print("Checksum:",esp.checksum_valid(msg,16))
        '''for k in range(len(msg)):
            print(k,struct.unpack('B',msg[k]))'''
        ser.write(msg)
    # echo anything we receive
    while ser.inWaiting() > 0:
        ch += ser.read()

ser.close()
