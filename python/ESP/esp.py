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

# message lengths
MSG_GPS_LEN = 16
MSG_CONTROL_LEN = 12
MSG_COMMAND_LEN = 12
MSG_SET_PID_LEN = 17

def message_gps():
    return MSG_GPS

def message_control():
    return MSG_CONTROL

def message_set_pid():
    return MSG_SET_PID

def message_command():
    return MSG_COMMAND

class espParser():
    def __init__(self):
        self.buffer = ''
        self.max_len = 1024# maxlength of buffer in bytes
        return
    ## Accept bytes from the serial port and search through for ESP messages
    def parseBytes(self,chars):
        self.buffer+=chars
        # index of buffer chars to remove
        idrm = -1
        # parse buffer
        numMsgs = 0
        msgs = []
        for k in range(1,len(self.buffer)-1):
            # check for header bytes
            (h1,h2,id0) = struct.unpack('BBB',self.buffer[k-1:k+2])
            if h1==ESP_HEADER1 and h2==ESP_HEADER2:
                # parse
                print(h1,h2,id0)
                numMsgs=numMsgs+1
                print(id0==MSG_GPS)
                if (id0==MSG_GPS):
                    finalInd = (k-1)+MSG_GPS_LEN+1
                    print(finalInd)
                    msgs.append(self.buffer[k-1:finalInd])
        # clear buffer
        self.buffer = self.buffer[finalInd:]
        # make sure buffer's not too long
        if len(self.buffer) > self.max_len:
            self.buffer = self.buffer[-(self.max_len):-1]
        return (numMsgs,msgs)
    def parseMessageBuffer(self,msgs):
        id = struct.unpack('B',msgs[0][2])
        if (id==MSG_GPS):
            (flag,lon,lat,time) = unpack_gps(msgs[0])
            return(flag,lon,lat,time)
        if (id==MSG_CONTROL):


def checksum_valid(msg,msg_len):
    chksum = 0
    chksum_msg = struct.unpack("B",msg[msg_len-1])[0]
    chksum = compute_checksum(msg,msg_len)
    # check
    return (chksum==chksum_msg)

def compute_checksum(msg,msg_len):
    chksum = 0
    for k in range(msg_len-1):
        chksum = chksum+struct.unpack("B",msg[k])[0]
    # convert to 8 bit
    chksum=chksum%256
    return chksum

def pack_gps(lon,lat,time):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_GPS)
    buf+=struct.pack('l',lon)
    buf+=struct.pack('l',lat)
    buf+=struct.pack('f',time)
    buf+=struct.pack('B',compute_checksum(buf,MSG_GPS_LEN))
    return buf

def pack_control(rudd,thro):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_CONTROL)
    buf+=struct.pack('f',rudd)
    buf+=struct.pack('f',thro)
    buf+=struct.pack('B',compute_checksum(buf,MSG_CONTROL_LEN))
    return buf

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

def unpack_control(buf):
    # header bytes
    rudd = struct.unpack('f',buf[3:7])[0]
    thro = struct.unpack('f',buf[7:11])[0]
    # checksum
    if not(checksum_valid(buf,MSG_CONTROL_LEN)):
        flag = -1
    else:
        flag = MSG_CONTROL_LEN
    return(flag,lon,lat,time)

def main():
    # test function
    msg = pack_gps(1901232309,-239239023,9.0)
    parser = espParser()
    (num,msgs) = parser.parseBytes(msg)
    return

if __name__ == "__main__":
    main()
