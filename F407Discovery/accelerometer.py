# *===========================================================================*
# * Name:       accelerometer.py                                              *
# * Version:    1.0                                                           *
# * Created_by: David Dudas - david.dudas@outlook.com                         *
# * Copyright:  David Dudas - david.dudas@outlook.com                         *
# *---------------------------------------------------------------------------*
# * Content: Driver library for LIS302DL                                      *
# *---------------------------------------------------------------------------*
# * Language: Python                                                          *
# * Compiler:                                                                 *
# * Target:   STM32F407 micropython 1.9.3                                     *
# *===========================================================================*

from pyb import Pin
from pyb import SPI

READWRITE_CMD = const(0x80) 
MULTIPLEBYTE_CMD = const(0x40)
LIS302DL_WHO_AM_I_ADDR = const(0x0f)
LIS302DL_CTRL_REG1_ADDR = const(0x20)
LIS302DL_CTRL_REG2_ADDR = const(0x21)

class LIS302DL:
    def __init__(self):
        self.cs_pin = Pin('PE3', Pin.OUT_PP, Pin.PULL_NONE)
        self.cs_pin.high()
        self.spi = SPI(1, SPI.MASTER, baudrate=328125, polarity=0, phase=1, bits=8)
        
        
    def rd(self, addr, nbytes):
        if nbytes > 1:
            addr |= READWRITE_CMD | MULTIPLEBYTE_CMD
        else:
            addr |= READWRITE_CMD
        self.cs_pin.low()
        self.spi.send(addr)
        buf = self.spi.send_recv(bytearray(nbytes * [0])) # read data, MSB first
        self.cs_pin.high()
        return buf

    def wr(self, addr, buf):
        if len(buf) > 1:
            addr |= MULTIPLEBYTE_CMD
        self.cs_pin.low()
        self.spi.send(addr)
        for b in buf:
            self.spi.send(b)
        self.cs_pin.high()

    def read_id(self):
        return self.rd(LIS302DL_WHO_AM_I_ADDR, 1)

    def init(self, init_param = 0x47, filter_param = 0x2D):
        self.wr(LIS302DL_CTRL_REG1_ADDR, bytearray([init_param]))
        self.wr(LIS302DL_CTRL_REG2_ADDR, bytearray([filter_param]))
        
    def getx(self):
        buf = (self.rd(0x29,1)[0])-1
        if buf > 127:
            return ((256-buf) * (-1)) / 5.81
        else:
            return buf / 5.81

    def gety(self):
        buf = (self.rd(0x2B,1)[0])-4
        if buf > 127:
            return ((256-buf) * (-1)) / 5.81
        else:
            return buf / 5.81
            
    def getz(self):
        buf = (self.rd(0x2D,1)[0])+8
        if buf > 127:
            return ((256-buf) * (-1)) / 5.81
        else:
            return buf / 5.81
        
        
        
        