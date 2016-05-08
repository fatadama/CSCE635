## @file Functions and object definitions for packing messages for interprocess communication

import os

## Nanomsg includes
from nanomsg_wrappers import set_wrapper_choice, get_default_for_platform
set_wrapper_choice(os.environ.get('NANOMSG_PY_TEST_WRAPPER',
                                  get_default_for_platform()))
from nanomsg import wrapper as nn_wrapper
from nanomsg import PAIR, AF_SP

## A mode message - swap teleoperation and pfields: msgid = mode
MSG_MODE = 1
## An EMILY position and pose message: msgid = egps
MSG_EMILY_GPS = 2
## A Target position and pose message: msgid = tgps
MSG_TARGET_GPS = 4
## A reference speed and heading message: msgid = rspd
MSG_SPEED_REF = 8

def ipc_pack_emily_gps(tNow,lat,lon,v,hdg):
    msg = b'egps,%f,%f,%f,%f,%f' % (tNow,lat,lon,v,hdg)
    return msg

def ipc_pack_target_gps(tNow,lat,lon,hdg):
    msg = b'tgps,%f,%f,%f,%f' % (tNow,lat,lon,hdg)
    return msg

def ipc_parse_msg(msg):
    vals = msg.split(',')
    #print(vals)
    if vals[0]=='mode':
        t = float(vals[1])
        mode = int(vals[2])
        return(MSG_MODE,t,mode)
    if vals[0]=='rspd':
        t = float(vals[1])
        v = float(vals[2])
        hdg = float(vals[3][:-1])
        return(MSG_SPEED_REF,t,v,hdg)
## TODO add other parsing later if needed

## Class for interprocess communication
#
# Currently only supports talking to the pfields program on a fixed socket
class nanomsgTalker():
    def __init__(self):
        ## Socket for IPC with pfields
        self.sock_pfield = nn_wrapper.nn_socket(AF_SP, PAIR)
        # bind socket
        nn_wrapper.nn_bind(self.sock_pfield, 'ipc://test')
        ## reference speed from pfields object on [0,1]
        self.v_ref = 0.0
        ## reference heading from pfields object (radians)
        self.hdg_ref = 0.0
        ## Flag that is set when we get a message from pfields
        self.new_ref = False
        ## Time associated with the pfields process
        self.t_ref = 0.0
    ## Read from IPC sockets
    def readSocks(self):
        # read from pfields Socket
        result = 1
        # empty the buffer
        while result > 0:
            result, buffer = nn_wrapper.nn_recv(self.sock_pfield, 1)
            if result > 0:
                #handle buffer
                msgout = ipc_parse_msg(bytes(buffer))
                if msgout[0]==MSG_SPEED_REF:
                    # set values
                    self.new_ref = True
                    self.t_ref = msgout[1]
                    self.v_ref = msgout[2]
                    self.hdg_ref = msgout[3]
    ## Write GPS out to sockets
    def writeSocks(self,tNow,lat,lon,v,hdg):
        # Pack GPS message
        msg = ipc_pack_emily_gps(tNow,lat,lon,v,hdg)
        # Write to pfields object
        nn_wrapper.nn_send(self.sock_pfield, msg, 1)
    ## Write a target position message to the pfields sockets
    #
    # For debugging and integration purposes only
    def writeTarget(self,tNow,tlat,tlon,thdg):
        msg = ipc_pack_target_gps(tNow,tlat,tlon,thdg)
        nn_wrapper.nn_send(self.sock_pfield, msg, 1)
    ## Destructor. Stop the nanomsg process
    def __del__(self):
        nn_wrapper.nn_term()
