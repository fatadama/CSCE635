import sys
# add path to serial protocol
sys.path.append('..\ESP')

# serial for talking to xbee
import serial
# time for running at fixed rates
import time

# dependencies for websocket package
import asyncio
# websocket package for IPC
import websockets

# import the bridge class definition
from xbee_bridge_state import xbee_bridge_state

class bridgeProcess():
    def __init__(self):
        self.state = xbee_bridge_state()
        self.time_10Hz = time.time()
        self.time_100Hz = time.time()
    def main_loop(self):
        tNow = time.time()
        if (tNow >= self.time_10Hz): # 10 Hz
            # execute the 10 Hz loop
            self.time_10Hz = self.time_10Hz + 0.1
            self.loop_10Hz(tNow)
        if (tNow >= self.time_100Hz): #100 Hz
            self.time_100Hz = self.time_100Hz + 0.01
            self.loop_100Hz(tNow)
    def loop_10Hz(self,tNow):
        return
    def loop_100Hz(self,tNow):
        return

def main():
    process = bridgeProcess()
    try:
        while True: # main loop
            process.main_loop()
    except KeyboardInterrupt:
        print("Leaving xbee_bridge")
        # shutdown function calls
    return 0

if __name__ == "__main__":
    main()
