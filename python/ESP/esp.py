import array
import struct

# header bytes
ESP_HEADER1 = 127
ESP_HEADER2 = 83

# message ids
MSG_GPS = 1
MSG_CONTROL = 2
MSG_COMMAND = 3
MSG_SET_PID = 4

# message lengths
MSG_GPS_LEN = 16
MSG_CONTROL_LEN = 12
MSG_COMMAND_LEN = 12
MSG_SET_PID_LEN = 17

def pack_int32(val):
    arr = array.array('B',[val & 255,(val>>8)&255,(val>>16)&255,(val>>24)&255])
    return arr

def unpack_int32(arr):
    val = arr[0] + (arr[1]<<8) + (arr[2]<<16) + (arr[3]<<24)
    return val

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

def esp_pack_gps(lon,lat,time):
    buf = bytes()
    buf+=struct.pack('%sB' % 3,ESP_HEADER1,ESP_HEADER2,MSG_GPS)
    arr = pack_int32(lon)
    for k in range(4):
        buf+=struct.pack('B',arr[k])
    arr = pack_int32(lat)
    for k in range(4):
        buf+=struct.pack('B',arr[k])
    buf+=struct.pack('f',time)
    buf+=struct.pack('B',compute_checksum(buf,MSG_GPS_LEN))
    return buf

def main():
    # test function
    esp_pack_gps(2301232309,-239239023,9.0)
    return

if __name__ == "__main__":
    main()
