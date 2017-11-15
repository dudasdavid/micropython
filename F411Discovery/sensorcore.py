# *===========================================================================*
# * Name:       sensorcore.py                                                 *
# * Version:    0.1                                                           *
# * Created_by: David Dudas - david.dudas@outlook.com                         *
# * Copyright:  David Dudas - david.dudas@outlook.com                         *
# *---------------------------------------------------------------------------*
# * Content: Driver library for L3GD20 and LSM303DLHC                         *
# *---------------------------------------------------------------------------*
# * Language: Python                                                          *
# * Compiler:                                                                 *
# * Target:   STM32F411 micropython 1.9.3                                     *
# *===========================================================================*

from pyb import Pin
from pyb import SPI
from pyb import I2C

# /* L3GD20 Registers ------------------------------------------------------------*/
L3GD20_WHO_AM_I_ADDR  = const(0x0F)  #/* device identification register */
L3GD20_CTRL_REG1_ADDR = const(0x20)  #/* Control register 1 */
L3GD20_CTRL_REG2_ADDR = const(0x21)  #/* Control register 2 */
L3GD20_CTRL_REG4_ADDR = const(0x23)  #/* Control register 4 */
L3GD20_OUT_TEMP_ADDR  = const(0x26)  #/* Out temp register */
L3GD20_OUT_X_L_ADDR   = const(0x28)  #/* Output Register X */
L3GD20_OUT_X_H_ADDR   = const(0x29)  #/* Output Register X */
L3GD20_OUT_Y_L_ADDR   = const(0x2A)  #/* Output Register Y */
L3GD20_OUT_Y_H_ADDR   = const(0x2B)  #/* Output Register Y */
L3GD20_OUT_Z_L_ADDR   = const(0x2C)  #/* Output Register Z */
L3GD20_OUT_Z_H_ADDR   = const(0x2D)  #/* Output Register Z */ 

READWRITE_CMD    = const(0x80) 
MULTIPLEBYTE_CMD = const(0x40)

# /* LSM303DLHC I2C address ------------------------------------------------------*/
ACC_I2C_ADDRESS = const(0x19)
MAG_I2C_ADDRESS = const(0x1E)
 
# /* Acceleration Registers */
LSM303DLHC_WHO_AM_I_ADDR   = const(0x0F)  #/* device identification register */
LSM303DLHC_CTRL_REG1_A     = const(0x20)  #/* Control register 1 acceleration */
LSM303DLHC_CTRL_REG2_A     = const(0x21)  #/* Control register 2 acceleration */
LSM303DLHC_CTRL_REG4_A     = const(0x23)  #/* Control register 4 acceleration */
LSM303DLHC_OUT_X_L_A       = const(0x28)  #/* Output Register X acceleration */
LSM303DLHC_OUT_X_H_A       = const(0x29)  #/* Output Register X acceleration */
LSM303DLHC_OUT_Y_L_A       = const(0x2A)  #/* Output Register Y acceleration */
LSM303DLHC_OUT_Y_H_A       = const(0x2B)  #/* Output Register Y acceleration */
LSM303DLHC_OUT_Z_L_A       = const(0x2C)  #/* Output Register Z acceleration */
LSM303DLHC_OUT_Z_H_A       = const(0x2D)  #/* Output Register Z acceleration */ 

# /* Magnetic field Registers */
LSM303DLHC_CRA_REG_M    = const(0x00)  #/* Control register A magnetic field */
LSM303DLHC_CRB_REG_M    = const(0x01)  #/* Control register B magnetic field */
LSM303DLHC_MR_REG_M     = const(0x02)  #/* Control register MR magnetic field */
LSM303DLHC_OUT_X_H_M    = const(0x03)  #/* Output Register X magnetic field */
LSM303DLHC_OUT_X_L_M    = const(0x04)  #/* Output Register X magnetic field */
LSM303DLHC_OUT_Z_H_M    = const(0x05)  #/* Output Register Z magnetic field */
LSM303DLHC_OUT_Z_L_M    = const(0x06)  #/* Output Register Z magnetic field */ 
LSM303DLHC_OUT_Y_H_M    = const(0x07)  #/* Output Register Y magnetic field */
LSM303DLHC_OUT_Y_L_M    = const(0x08)  #/* Output Register Y magnetic field */

LSM303DLHC_SR_REG_M     = const(0x09)  #/* Status Register magnetic field */
LSM303DLHC_IRA_REG_M    = const(0x0A)  #/* IRA Register magnetic field */
LSM303DLHC_IRB_REG_M    = const(0x0B)  #/* IRB Register magnetic field */
LSM303DLHC_IRC_REG_M    = const(0x0C)  #/* IRC Register magnetic field */

LSM303DLHC_TEMP_OUT_H_M = const(0x31)  #/* Temperature Register magnetic field */
LSM303DLHC_TEMP_OUT_L_M = const(0x32)  #/* Temperature Register magnetic field */

def s16(value):
    return -(value & 0x8000) | (value & 0x7fff)

class L3GD20:
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
        return (self.rd(L3GD20_WHO_AM_I_ADDR, 1)[0])
        
    def init(self, reg1_param = 0x3F, reg2_param = 0x00, reg4_param = 0x10):
        self.wr(L3GD20_CTRL_REG1_ADDR, bytearray([reg1_param]))
        self.wr(L3GD20_CTRL_REG2_ADDR, bytearray([reg2_param]))
        self.wr(L3GD20_CTRL_REG4_ADDR, bytearray([reg4_param]))

    def getx(self):
        buf = (self.rd(L3GD20_OUT_X_L_ADDR,2))
        num = buf[1] << 8 | buf[0]
        return (s16(num)*17.50*0.001)
        
    def gety(self):
        buf = (self.rd(L3GD20_OUT_Y_L_ADDR,2))
        num = buf[1] << 8 | buf[0]
        return (s16(num)*17.50*0.001)
        
    def getz(self):
        buf = (self.rd(L3GD20_OUT_Z_L_ADDR,2))
        num = buf[1] << 8 | buf[0]
        return (s16(num)*17.50*0.001)
        
    def gettemp(self):
        buf = (self.rd(L3GD20_OUT_TEMP_ADDR,1)[0])
        return s16(buf)
        
class LSM303DLHC:
    def __init__(self):
        self.i2c = I2C(1, I2C.MASTER, baudrate=100000)
        
    def init(self, reg1_param = 0x47, reg2_param = 0x90, reg4_param = 0x08):
        self.i2c.mem_write(bytearray([reg1_param]), ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A)
        self.i2c.mem_write(bytearray([reg2_param]), ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A)
        self.i2c.mem_write(bytearray([reg4_param]), ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A)
        
    def read_id(self):
        buf = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR)
        return buf
        
    def getx(self):
        bufL = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A)
        bufH = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A)
        num = bufH[0] << 8 | bufL[0]
        return (s16(num) * 1 / 16.0 * 0.00980665)
    
    def gety(self):
        bufL = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_A)
        bufH = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_A)
        num = bufH[0] << 8 | bufL[0]
        return (s16(num) * 1 / 16.0 * 0.00980665)
        
    def getz(self):
        bufL = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_A)
        bufH = self.i2c.mem_read(1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_A)
        num = bufH[0] << 8 | bufL[0]
        return (s16(num) * 1 / 16.0 * 0.00980665)
