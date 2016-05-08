## @file Functions for packing messages for interprocess communication

import struct

## A mode message - swap teleoperation and pfields: msgid = mode
MSG_MODE = 1
## An EMILY position and pose message: msgid = egps
MSG_EMILY_GPS = 2
## A Target position and pose message: msgid = tgps
MSG_TARGET_GPS = 4
## A reference speed and heading message: msgid = rspd
MSG_SPEED_REF = 8

def ipc_pack_emily_gps(tNow,lat,lon,v,hdg):
    msg = b'egps,%f,%f,%f,%f,%f' % (time,lat,lon,v,hdg)
    return msg

def ipc_pack_target_gps(tNow,lat,lon,hdg):
    msg = b'tgps,%f,%f,%f,%f' % (time,lat,lon,hdg)
    return msg

def ipc_parse_msg(msg):
    vals = msg.split(',')
    if vals[0]=='mode':
        t = float(vals[1])
        mode = int(vals[2])
        return(MSG_MODE,t,mode)
    if vals[0]=='rspd':
        t = float(vals[1])
        v = float(vals[2])
        hdg = float(vals[3])
        return(MSG_SPEED_REF,t,v,hdg)
