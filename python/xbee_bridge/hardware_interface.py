## @file interface to an XBee at  a low level, or use UDP to emulate an XBee

import serial
import socket

UDP_SERVER_RX = 49152
UDP_SERVER_TX = 49153

class hardware_interface:
    def __init__(self,port,SIL=False,rate='9600',groundstation=True):
        self.groundstation = groundstation
        self.SIL = SIL
        if self.SIL==False:
            self.ser = serial.Serial()
            ser.baudrate = rate
            ser.port = port
        else:
            self.sock_send = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
            self.sock_recv = socket.socket( socket.AF_INET, socket.SOCK_DGRAM )
            # set timeout
            self.sock_send.settimeout(0.01)
            self.sock_recv.settimeout(0.01)
    def __del__(self):
        if self.SIL==False:
            if self.ser.is_open():
                self.ser.close()
        else:
            self.sock_send.close()
            self.sock_recv.close()
    ## open the serial port
    def start(self):
        if self.SIL==False:
            self.ser.open()
        else:
            if self.groundstation:
                self.sock_send.connect(('127.0.0.1',UDP_SERVER_TX))
                self.sock_recv.bind(('127.0.0.1',UDP_SERVER_RX))
            else:
                self.sock_send.connect(('127.0.0.1',UDP_SERVER_RX))
                self.sock_recv.bind(('127.0.0.1',UDP_SERVER_TX))
    def write(self,buff):
        if self.SIL==False:
            self.ser.write(buff)
        else:
            if self.groundstation:
                self.sock_send.sendto(buff,('127.0.0.1', UDP_SERVER_TX))
            else:
                self.sock_send.sendto(buff,('127.0.0.1', UDP_SERVER_RX))
    ## read from serial port
    def read(self):
        if self.SIL==False:
            ch = ''
            while self.ser.inWaiting() > 0:
                ch += self.ser.read()
            return ch
        else:
            try:
                (ch,addr) = self.sock_recv.recvfrom(4096)
                return ch
            except socket.timeout:
                return ''
