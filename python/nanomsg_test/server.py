import sys
sys.path.append('../xbee_bridge/')
import os
import time

import ipcPacker

from nanomsg_wrappers import set_wrapper_choice, get_default_for_platform
set_wrapper_choice(os.environ.get('NANOMSG_PY_TEST_WRAPPER',
                                  get_default_for_platform()))
#from nanomsg import Socket, PAIR, AF_SP

'''
@ file
Simple file to send test message for the pfields_realtime code to test it
'''

'''
# Here's another way to do it
s1 = Socket(PAIR)
s1.bind('ipc://test')
while True:
    try:
        msg = b'egps,%f,%f,%f,%f,%f'  % (8.272783,89.2392839,-293.82982393,8.23728323,1.023823782)
        s1.send(msg)
        # now receive
        out = s1.recv()
        if len(out) > 0:
            print(out)
        time.sleep(1.0)
    except KeyboardInterrupt:
        s1.close()
'''

from nanomsg import wrapper as nn_wrapper
from nanomsg import PAIR, AF_SP

s1 = nn_wrapper.nn_socket(AF_SP, PAIR)
nn_wrapper.nn_bind(s1, 'ipc://test')

counter = 0
while counter < 50:
    try:
        msg = b'egps,%f,%f,%f,%f,%f'  % (8.272783,89.2392839,-293.82982393,8.23728323,1.023823782)
        nn_wrapper.nn_send(s1, msg, 1)
        result = 1
        # empty the buffer
        while result > 0:
            result, buffer = nn_wrapper.nn_recv(s1, 1)
            if result > 0:
                #print(bytes(buffer))
                msgContents = ipcPacker.ipc_parse_msg(bytes(buffer))
                if msgContents[0] == ipcPacker.MSG_SPEED_REF:
                    print(msgContents[1],msgContents[2],msgContents[3])
        time.sleep(0.02)
        counter=counter+1
    except KeyboardInterrupt:
        print("Exiting")
        break;

nn_wrapper.nn_term()
