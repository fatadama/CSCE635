import array
import struct

# header bytes
ESP_HEADER1 = 127
ESP_HEADER2 = 83

# message ids
MSG_GPS = 1
MSG_CONTROL = 2
MSG_COMMAND = 4
MSG_SET_PID = 8
MSG_GPS_POS = 16

# message lengths
MSG_GPS_LEN = 16
MSG_CONTROL_LEN = 12
MSG_COMMAND_LEN = 12
MSG_SET_PID_LEN = 17
MSG_GPS_POS_LEN = 25

def message_gps():
    return MSG_GPS

def message_control():
    return MSG_CONTROL

def message_set_pid():
    return MSG_SET_PID

def message_command():
    return MSG_COMMAND

## Parser class for handling messages
class espParser():
    ## Initialization function. Sets default max buffer length to 1024.
    def __init__(self):
        self.buffer = ''
        self.max_len = 1024# maxlength of buffer in bytes
        return
    ## Accept bytes from the serial port and search through for ESP messages.
    #
    # Stores a string as a class member. Whenever a message is parsed, delete the buffer up to that point.
    # @param[in] strings from serial.read()
    # @param[out] numMsgs the number of messages read from the buffer
    # @param[out] msgs a list of messages in the buffer
    def parseBytes(self,chars):
        self.buffer+=chars
        # index of buffer chars to remove
        idrm = 0
        # parse buffer
        numMsgs = 0
        msgs = []
        for k in range(1,len(self.buffer)-1):
            # check for header bytes
            (h1,h2,id0) = struct.unpack('BBB',self.buffer[k-1:k+2])
            if h1==ESP_HEADER1 and h2==ESP_HEADER2:
                # parse
                print(h1,h2,id0)
                # check that there are enough bites remaining
                valid = False
                if id0 == MSG_GPS:
                    if len(self.buffer[k-1:]) >= MSG_GPS_LEN:
                        finalInd = (k-1)+MSG_GPS_LEN+1
                        valid = True
                if id0 == MSG_CONTROL:
                    if len(self.buffer[k-1:]) >= MSG_CONTROL_LEN:
                        finalInd = (k-1)+MSG_CONTROL_LEN+1
                        valid = True
                if id0 == MSG_COMMAND:
                    if len(self.buffer[k-1:]) >= MSG_COMMAND_LEN:
                        finalInd = (k-1)+MSG_COMMAND_LEN+1
                        valid = True
                if id0 == MSG_SET_PID:
                    if len(self.buffer[k-1:]) >= MSG_SET_PID_LEN:
                        finalInd = (k-1)+MSG_SET_PID_LEN+1
                        valid = True
                if id0 == MSG_GPS_POS:
                    if len(self.buffer[k-1:]) >= MSG_GPS_POS_LEN:
                        finalInd = (k-1)+MSG_GPS_POS_LEN+1
                        valid = True
                if valid:
                    numMsgs=numMsgs+1
                    msgs.append(self.buffer[k-1:finalInd])
                # make sure we always remove the highest index that we've parsed
                if idrm <= finalInd:
                    idrm = finalInd
        numMsgs = len(msgs)
        # clear buffer
        self.buffer = self.buffer[idrm:]
        # make sure buffer's not too long
        if len(self.buffer) > self.max_len:
            self.buffer = self.buffer[-(self.max_len):-1]
        return (numMsgs,msgs)
    ## Parse the first in a list of messages. Returns output dependent on the message ID.
    #
    # Probably don't use this function, it isn't a very good solution. See serial_test_debugging.py for a reasonable approach.
    def parseMessageBuffer(self,msgs):
        id = struct.unpack('B',msgs[0][2])
        if (id==MSG_GPS):
            (flag,lon,lat,time) = unpack_gps(msgs[0])
            return(flag,lon,lat,time)
        if (id==MSG_CONTROL):
            (flag,rudd,thro) = unpack_control(msgs[0])
            return(flag,rudd,thro)
        if (id==MSG_COMMAND):
            (flag,hdg,speed) = unpack_command(msgs[0])
            return(flag,hdg,speed)
        if (id==MSG_SET_PID):
            (flag,ch,Kp,Ki,Kd) = unpack_set_pid(msgs[0])
            return(flag,ch,Kp,Ki,Kd)
        if (id==MSG_GPS_POS):
            (flag,lon,lat,t,v,hdg) = unpack_set_pid(msgs[0])
            return(flag,lon,lat,t,v,hdg)

## Return true if a checksum byte in a message is the correct value
def checksum_valid(msg,msg_len):
    chksum = 0
    chksum_msg = struct.unpack("B",msg[msg_len-1])[0]
    chksum = compute_checksum(msg,msg_len)
    # check
    return (chksum==chksum_msg)

## Compute a checksum for a message of specified length
def compute_checksum(msg,msg_len):
    chksum = 0
    for k in range(msg_len-1):
        chksum = chksum+struct.unpack("B",msg[k])[0]
    # convert to 8 bit
    chksum=chksum%256
    return chksum

## Pack a GPS command message
# @param[in] lon the longitude in (degrees x 10^7) as a python int
# @param[in] lat the latitude in (degrees x 10^7) as a python int
# @param[in] time the time as a python float. Not used by Arduino.
# @param[out] the message. Write to the serial port using write()
def pack_gps(lon,lat,time):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_GPS)
    buf+=struct.pack('l',lon)
    buf+=struct.pack('l',lat)
    buf+=struct.pack('f',time)
    buf+=struct.pack('B',compute_checksum(buf,MSG_GPS_LEN))
    return buf

## Pack a low-level control message
# @param[in] rudd the rudder servo command mapped from [-1,1] as a python float
# @param[in] thro the throttle servo command mapped from [0,1] as a python float
# @param[out] the message. Write to the serial port using write()
def pack_control(rudd,thro):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_CONTROL)
    buf+=struct.pack('f',rudd)
    buf+=struct.pack('f',thro)
    buf+=struct.pack('B',compute_checksum(buf,MSG_CONTROL_LEN))
    return buf

## Pack a high-level control message
# @param[in] hdg the relative heading in radians from [-pi, pi] as a python float
# @param[in] speed the speed on [0,1] or range as a distance in meters. Have not decided which yet.
# @param[out] the message. Write to the serial port using write()
def pack_command(hdg,speed):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_COMMAND)
    buf+=struct.pack('f',hdg)
    buf+=struct.pack('f',speed)
    buf+=struct.pack('B',compute_checksum(buf,MSG_COMMAND_LEN))
    return buf

## Pack a PID command message
# @param[in] ch the channel (0 == rudder, 1 == throttle) for which to set PID values as a python int
# @param[in] Kp the proportional gain as a float
# @param[in] Ki the integral gain as a float
# @param[in] Kd the derivative gain as a float
# @param[out] the message. Write to the serial port using write()
def pack_set_pid(ch,Kp,Ki,Kd):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_SET_PID)
    buf+=struct.pack('B',ch)
    buf+=struct.pack('f',Kp)
    buf+=struct.pack('f',Ki)
    buf+=struct.pack('f',Kd)
    buf+=struct.pack('B',compute_checksum(buf,MSG_SET_PID_LEN))
    return buf

## Unpack a combined GPS/heading/speed command message
# @param[in] lon the longitude in (degrees x 10^7) as a python int
# @param[in] lat the latitude in (degrees x 10^7) as a python int
# @param[in] time the time as a python float. Not used by Arduino.
# @param[in] v the speed (m/s)
# @param[in] hdg the heading in radians
# @param[in] status the status byte. 0 == no GPS lock, 1 == GPS lock
# @param[out] the message. Write to the serial port using write()
def pack_gps_pos(lon,lat,time,v,hdg):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_GPS_POS)
    buf+=struct.pack('l',lon)
    buf+=struct.pack('l',lat)
    buf+=struct.pack('f',time)
    buf+=struct.pack('f',v)
    buf+=struct.pack('f',hdg)
    buf+=struct.pack('B',status)
    buf+=struct.pack('B',compute_checksum(buf,MSG_GPS_POS_LEN))
    return buf

## Unpack a GPS message
# @param[in] The message from serial.read()
# @param[out] flag (-1 if the checksum is invalid; else, the number of bytes read)
# @param[out] lon the longitude in (degrees x 10^7) as a python int
# @param[out] lat the latitude in (degrees x 10^7) as a python int
# @param[out] time the time as a python float.
def unpack_gps(buf):
    # header bytes
    lon = struct.unpack('l',buf[3:7])[0]
    lat = struct.unpack('l',buf[7:11])[0]
    time = struct.unpack('f',buf[11:15])[0]
    # checksum
    if not(checksum_valid(buf,MSG_GPS_LEN)):
        flag = -1
    else:
        flag = MSG_GPS_LEN
    return(flag,lon,lat,time)

## Unpack a low-level control message
# @param[in] The message from serial.read()
# @param[out] flag (-1 if the checksum is invalid; else, the number of bytes read)
# @param[out] rudd the rudder command as a float on [-1, 1]
# @param[out] thro the throttle command as a float on [0,1]
def unpack_control(buf):
    # header bytes
    rudd = struct.unpack('f',buf[3:7])[0]
    thro = struct.unpack('f',buf[7:11])[0]
    # checksum
    if not(checksum_valid(buf,MSG_CONTROL_LEN)):
        flag = -1
    else:
        flag = MSG_CONTROL_LEN
    return(flag,rudd,thro)

## Unpack a high-level control message
# @param[in] The message from serial.read()
# @param[out] flag (-1 if the checksum is invalid; else, the number of bytes read)
# @param[out] hdg the relative heading EMILY is going to
# @param[out] speed either the range (metres) or the speed ([0,1]) at which EMILY is going.
def unpack_command(buf):
    hdg=struct.unpack('f',buf[3:7])[0]
    speed=struct.unpack('f',buf[7:11])[0]
    # checksum
    if not(checksum_valid(buf,MSG_COMMAND_LEN)):
        flag = -1
    else:
        flag = MSG_COMMAND_LEN
    return(flag,hdg,speed)

## Unpack a PID confirmation message
# @param[in] The message from serial.read()
# @param[out] ch the channel (0 == rudder, 1 == throttle) for which to set PID values as a python int
# @param[out] Kp the proportional gain as a float
# @param[out] Ki the integral gain as a float
# @param[out] Kd the derivative gain as a float
def unpack_set_pid(buf):
    ch=struct.unpack('B',buf[3])[0]
    Kp=struct.unpack('f',buf[4:8])[0]
    Ki=struct.unpack('f',buf[8:12])[0]
    Kd=struct.unpack('f',buf[12:16])[0]
    # checksum
    if not(checksum_valid(buf,MSG_SET_PID_LEN)):
        flag = -1
    else:
        flag = MSG_SET_PID_LEN
    return(flag,ch,Kp,Ki,Kd)

## Unpack a GPS/speed/heading message
# @param[in] The message from serial.read()
# @param[out] flag (-1 if the checksum is invalid; else, the number of bytes read)
# @param[out] lon the longitude in degrees, x 10^7, as a python int
# @param[out] lat the latitude in degrees, x 10^7, as a python int
# @param[out] time the time as a python float.
# @param[out] v the speed in m/s as a python float.
# @param[out] hsg the heading in radians as a python float.
# @param[out] status the status byte
def unpack_gps_pos(buf):
    # header bytes
    lon = struct.unpack('l',buf[3:7])[0]
    lat = struct.unpack('l',buf[7:11])[0]
    time = struct.unpack('f',buf[11:15])[0]
    v = struct.unpack('f',buf[15:19])[0]
    hdg = struct.unpack('f',buf[19:23])[0]
    status = struct.unpack('B',buf[23])[0]
    # checksum
    if not(checksum_valid(buf,MSG_GPS_POS_LEN)):
        flag = -1
    else:
        flag = MSG_GPS_POS_LEN
    return(flag,lon,lat,time,v,hdg,status)

def main():
    # test function
    msg = pack_gps(1901232309,-239239023,9.0)
    parser = espParser()
    (num,msgs) = parser.parseBytes(msg)
    return

if __name__ == "__main__":
    main()
