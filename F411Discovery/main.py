# *===========================================================================*
# * Name:       main.py                                                       *
# * Version:    1.0                                                           *
# * Created_by: David Dudas - david.dudas@outlook.com                         *
# * Copyright:  David Dudas - david.dudas@outlook.com                         *
# *---------------------------------------------------------------------------*
# * Content:                                                                  *
# *---------------------------------------------------------------------------*
# * Language: Python                                                          *
# * Compiler:                                                                 *
# * Target:   STM32F411 micropython 1.9.3                                     *
# *===========================================================================*

import pyb
import sensorcore
import filterToolkit
import time
import math
from pyb import Pin, Timer, ExtInt, Switch, UART
import threading

uart = UART(2,9600, timeout=1000)

switch = Switch()
lastPressTime = time.time()

l3gd20 = sensorcore.L3GD20()
l3gd20.initGyro()

lsm303dlhc = sensorcore.LSM303DLHC()
lsm303dlhc.initAcc()
lsm303dlhc.initAccInt() # enable click interrupt
lsm303dlhc.initMag()

roll, pitch, heading = 0,0,0

rawXGyro, rawYGyro, rawZGyro = 0,0,0
rawXAcc,  rawYAcc,  rawZAcc  = 0,0,0
rawXMag,  rawYMag,  rawZMag  = 0,0,0

filtXGyro, filtYGyro, filtZGyro = 0,0,0
filtXAcc,  filtYAcc,  filtZAcc  = 0,0,0
filtXMag,  filtYMag,  filtZMag  = 0,0,0

magXNormFiltered, magYNormFiltered, magZNormFiltered = 0,0,0

magXMax = 557.7
magXMin = -555.5
magYMax = 502.2
magYMin = -631.1
magZMax = 472.4
magZMin = -554.9
initialCompassCalib = 0 # if this is enabled it will not use the above calibration data

PI = 3.14156

ledMode = 0

ledLeft = Pin('PD12')
ledRight = Pin('PD14')
ledUp = Pin('PD13')
ledDown = Pin('PD15')

tim = Timer(4,freq=1000)

chLeft = tim.channel(1,Timer.PWM, pin=ledLeft)
chRight = tim.channel(3,Timer.PWM, pin=ledRight)
chUp = tim.channel(2,Timer.PWM, pin=ledUp)
chDown = tim.channel(4,Timer.PWM, pin=ledDown)

headingOffset = 0
originalHeading = 0

def interruptCallback():
    global headingOffset, originalHeading
    headingOffset = originalHeading
    
    # print(headingOffset)
    print("Interrupt was received")
    
callback = lambda e: interruptCallback()
ext = ExtInt(Pin('PE4'), ExtInt.IRQ_RISING, Pin.PULL_NONE, callback)

def calcPitchRollHeading(xAcc,yAcc,zAcc, xMag, yMag, zMag):
    normAcc = math.sqrt((xAcc*xAcc)+(yAcc*yAcc)+(zAcc*zAcc))
      
    sinRoll = yAcc/normAcc
    cosRoll = math.sqrt(1.0-(sinRoll * sinRoll))
    sinPitch = -xAcc/normAcc
    cosPitch = math.sqrt(1.0-(sinPitch * sinPitch))
    
    # //Roll & Pitch Equations
    # roll_alt  = (math.atan2(yAcc, zAcc)*180.0)/PI
    # pitch_alt = (math.atan2(-xAcc, math.sqrt(yAcc*yAcc + zAcc*zAcc))*180.0)/PI
    
    # print("sinRoll:%.3f cosRoll:%.3f" % (sinRoll,cosRoll))
    # print("roll:%.3f pitch:%.3f" % (roll_alt,pitch_alt))
    
    if (sinRoll>0):
      if (cosRoll>0):
        calcRoll = math.acos(cosRoll)*180/PI
      else:
        calcRoll = math.acos(cosRoll)*180/PI + 180
    else:
      if (cosRoll>0):
        calcRoll = math.acos(cosRoll)*180/PI + 360
      else:
        calcRoll = math.acos(cosRoll)*180/PI + 180
     
    if (sinPitch>0):
      if (cosPitch>0):
        calcPitch = math.acos(cosPitch)*180/PI
      else:
        calcPitch = math.acos(cosPitch)*180/PI + 180
    else:
      if (cosPitch>0):
        calcPitch = math.acos(cosPitch)*180/PI + 360
      else:
        calcPitch = math.acos(cosPitch)*180/PI + 180

    if (calcRoll >=360):
        calcRoll = 360 - calcRoll
    if (calcPitch >=360):
        calcPitch = 360 - calcPitch

    magXNormFilteredTilted = xMag*cosPitch+zMag*sinPitch
    magYNormFilteredTilted = xMag*sinRoll*sinPitch+yMag*cosRoll-zMag*sinRoll*cosPitch
        
    calcHeading = (math.atan2(magYNormFilteredTilted,magXNormFilteredTilted)*180)/PI
    calcHeading += 180
    
    global originalHeading, headingOffset
    originalHeading = calcHeading
    calcHeading -= headingOffset
        
    if (calcHeading < 0): calcHeading = calcHeading + 360;
        
    return calcRoll, calcPitch, calcHeading

def task1():
    global initialCompassCalib, ledMode
    global magXMax, magYMax, magZMax
    global magXMin, magYMin, magZMin
    global magXNormFiltered, magYNormFiltered, magZNormFiltered
    global filtXAcc, filtYAcc, filtZAcc

    while(1):
        time.sleep(0.01)
        
        rawXGyro = l3gd20.getXGyro()
        rawYGyro = l3gd20.getYGyro()
        rawZGyro = l3gd20.getZGyro()
        
        rawXAcc = lsm303dlhc.getXAcc()
        rawYAcc = lsm303dlhc.getYAcc()
        rawZAcc = lsm303dlhc.getZAcc()
        
        rawXMag = lsm303dlhc.getXMag()
        rawYMag = lsm303dlhc.getYMag()
        rawZMag = lsm303dlhc.getZMag()
        
        if (initialCompassCalib == 1):
            magXMax = rawXMag
            magXMin = rawXMag
            magYMax = rawYMag
            magYMin = rawYMag
            magZMax = rawZMag
            magZMin = rawZMag
            initialCompassCalib = 0
        
        if (rawXMag > magXMax): magXMax = rawXMag
        if (rawYMag > magYMax): magYMax = rawYMag
        if (rawZMag > magZMax): magZMax = rawZMag
        if (rawXMag < magXMin): magXMin = rawXMag
        if (rawYMag < magYMin): magYMin = rawYMag
        if (rawZMag < magZMin): magZMin = rawZMag
        
        # print("xmin:%.1f xmax:%.1f ymin:%.1f ymax:%.1f zmin:%.1f zmax:%.1f" % (magXMin,magXMax,magYMin, magYMax, magZMin, magZMax))

        if (magXMax - magXMin) != 0: magXNorm = (rawXMag - magXMin) / (magXMax - magXMin) * 2 - 1.0
        else: magXNorm = 0
        if (magYMax - magYMin) != 0: magYNorm = (rawYMag - magYMin) / (magYMax - magYMin) * 2 - 1.0
        else: magYNorm = 0
        if (magZMax - magZMin) != 0: magZNorm = (rawZMag - magZMin) / (magZMax - magZMin) * 2 - 1.0
        else: magZNorm = 0
        
        magXNormFiltered = filterToolkit.filter2(magXNormFiltered, magXNorm, 0.3)
        magYNormFiltered = filterToolkit.filter2(magYNormFiltered, magYNorm, 0.3)
        magZNormFiltered = filterToolkit.filter2(magZNormFiltered, magZNorm, 0.3)
        
        # print("x:%.1f y:%.1f z:%.1f" % (rawXGyro,rawYGyro,rawZGyro))
        # print("x:%.1f y:%.1f z:%.1f" % (rawXAcc,rawYAcc,rawZAcc))
        # print("x:%.1f y:%.1f z:%.1f\n" % (rawXMag,rawYMag,rawZMag))
        
        filtXAcc = filterToolkit.filter2(filtXAcc,rawXAcc,0.2)
        filtYAcc = filterToolkit.filter2(filtYAcc,rawYAcc,0.2)
        filtZAcc = filterToolkit.filter2(filtZAcc,rawZAcc,0.2)
        
        roll, pitch, heading = calcPitchRollHeading(filtXAcc,filtYAcc,filtZAcc,magXNormFiltered,magYNormFiltered,magZNormFiltered)
        
        print("roll:%.1f pitch:%.1f heading:%.1f" % (roll,pitch,heading))

        if (ledMode == 0):   # compass mode
            if (heading >=0 and heading <90):
                chDown.pulse_width_percent((90 - heading)*0.5)
                chRight.pulse_width_percent((heading)*0.5)
                chUp.pulse_width_percent(0)
                chLeft.pulse_width_percent(0)
            elif (heading >=90 and heading <180):
                chRight.pulse_width_percent((90 - (heading - 90))*0.5)
                chUp.pulse_width_percent((heading - 90)*0.5)
                chDown.pulse_width_percent(0)
                chLeft.pulse_width_percent(0)
            elif (heading >=180 and heading <270):
                chUp.pulse_width_percent((90 - (heading - 180))*0.5)
                chLeft.pulse_width_percent((heading - 180)*0.5)
                chDown.pulse_width_percent(0)
                chRight.pulse_width_percent(0)
            elif (heading >=270 and heading <360):
                chLeft.pulse_width_percent((90 - (heading - 270))*0.5)
                chDown.pulse_width_percent((heading - 270)*0.5)
                chUp.pulse_width_percent(0)
                chRight.pulse_width_percent(0)
            else:
                print("ivalid heading value: %.1f" % heading)
        
        elif (ledMode == 1): # pitch roll mode
            if (roll >= 0):
                chRight.pulse_width_percent(int(roll/3.0))
                chLeft.pulse_width_percent(0)
            else:
                chLeft.pulse_width_percent(int(-roll/3.0))
                chRight.pulse_width_percent(0)
            
            if (pitch >= 0):
                chUp.pulse_width_percent(int(pitch/3.0))
                chDown.pulse_width_percent(0)
            else:
                chDown.pulse_width_percent(int(-pitch/3.0))
                chUp.pulse_width_percent(0)
        
        if (switch.value() == True and time.time() > lastPressTime):
            lastPressTime = time.time()
            ledMode += 1
            if (ledMode >= 2):
                ledMode = 0
        
        
def task2():
    while 1:
        message = uart.readline()
        if message != None:print(message)
        
threads = []
t = threading.Thread(target=task1)
threads.append(t)
t.start()

t = threading.Thread(target=task2)
threads.append(t)
t.start()
        