# *===========================================================================*
# * Name:       main.py                                                       *
# * Version:    1.1                                                           *
# * Created_by: David Dudas - david.dudas@outlook.com                         *
# * Copyright:  David Dudas - david.dudas@outlook.com                         *
# *---------------------------------------------------------------------------*
# * Content:                                                                  *
# *---------------------------------------------------------------------------*
# * Language: Python                                                          *
# * Compiler:                                                                 *
# * Target:   STM32F407 micropython 1.9.3                                     *
# *===========================================================================*

import pyb
import accelerometer
import filterToolkit
import time
import math
from pyb import UART
from pyb import Pin, Timer

uart = UART(2,9600, timeout=10) # UART2 -> Rx PA3, Tx PA2
acc = accelerometer.LIS302DL()
acc.init()

rawX, rawY, rawZ = 0,0,0
filteredX, filteredY, filteredZ = 0,0,0
PI = 3.14156

ledLeft = Pin('PD12')
ledRight = Pin('PD14')
ledUp = Pin('PD13')
ledDown = Pin('PD15')

tim = Timer(4,freq=1000)

chLeft = tim.channel(1,Timer.PWM, pin=ledLeft)
chRight = tim.channel(3,Timer.PWM, pin=ledRight)
chUp = tim.channel(2,Timer.PWM, pin=ledUp)
chDown = tim.channel(4,Timer.PWM, pin=ledDown)

cycleCounter = 0;
cycleCounterPrev = 0;

uart.write("init complete\r\n")

def calcPitchAndRoll(x,y,z):
    normAcc = math.sqrt((x*x)+(y*y)+(z*z))
      
    sinRoll = -y/normAcc
    cosRoll = math.sqrt(1.0-(sinRoll * sinRoll))
    sinPitch = x/normAcc
    cosPitch = math.sqrt(1.0-(sinPitch * sinPitch))
      
    if (sinRoll>0):
      if (cosRoll>0):
        roll = math.acos(cosRoll)*180/PI
      else:
        roll = math.acos(cosRoll)*180/PI + 180
    else:
      if (cosRoll>0):
        roll = math.acos(cosRoll)*180/PI + 360
      else:
        roll = math.acos(cosRoll)*180/PI + 180
     
    if (sinPitch>0):
      if (cosPitch>0):
        pitch = math.acos(cosPitch)*180/PI
      else:
        pitch = math.acos(cosPitch)*180/PI + 180
    else:
      if (cosPitch>0):
        pitch = math.acos(cosPitch)*180/PI + 360
      else:
        pitch = math.acos(cosPitch)*180/PI + 180

    if (roll >=360):
        roll = 360 - roll
    if (pitch >=360):
        pitch = 360 - pitch

    return roll, pitch

while(1):
    time.sleep(0.01)
    cycleCounter+=1
    
    rawX = acc.getx()
    rawY = acc.gety()
    rawZ = acc.getz()
    # uart.write("x:%.1f y:%.1f z:%.1f" % (rawX,rawY,rawZ))
    # uart.write("x:%.1f y:%.1f z:%.1f" % (rawX,rawY,rawZ))
    
    filteredX = filterToolkit.filter2(filteredX,rawX,0.2)
    filteredY = filterToolkit.filter2(filteredY,rawY,0.2)
    filteredZ = filterToolkit.filter2(filteredZ,rawZ,0.2)
    # uart.write(" - x:%.1f y:%.1f z:%.1f" % (filteredX,filteredY,filteredZ))
    # uart.write(" - x:%.1f y:%.1f z:%.1f" % (filteredX,filteredY,filteredZ))
    
    roll, pitch = calcPitchAndRoll(filteredX,filteredY,filteredZ)
    # uart.write("roll:%.1f pitch:%.1f\r\n" % (roll,pitch))
    # uart.write("roll:%.1f pitch:%.1f\r\n" % (roll,pitch))
    
    if (roll >=0):
        chLeft.pulse_width_percent(int(roll/2))
        chRight.pulse_width_percent(0)
    else:
        chRight.pulse_width_percent(int(-roll/2))
        chLeft.pulse_width_percent(0)
    
    if (pitch >=0):
        chDown.pulse_width_percent(int(pitch/2))
        chUp.pulse_width_percent(0)
    else:
        chUp.pulse_width_percent(int(-pitch/2))
        chDown.pulse_width_percent(0)
    
    if (cycleCounter-cycleCounterPrev) >= 100:
        cycleCounterPrev = cycleCounter
        message = uart.readline()
        if message != None:print(message)
        if message == b'roll\n':
            uart.write("%.1f" % roll)
        elif message == b'pitch\n':
            uart.write("%.1f" % pitch)
    