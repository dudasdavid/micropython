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
from pyb import Pin, Timer

l3gd20 = sensorcore.L3GD20()
l3gd20.initGyro()

lsm303dlhc = sensorcore.LSM303DLHC()
lsm303dlhc.initAcc()
lsm303dlhc.initMag()

rawXGyro, rawYGyro, rawZGyro = 0,0,0
rawXAcc,  rawYAcc,  rawZAcc  = 0,0,0
rawXMag,  rawYMag,  rawZMag  = 0,0,0

filtXGyro, filtYGyro, filtZGyro = 0,0,0
filtXAcc,  filtYAcc,  filtZAcc  = 0,0,0
filtXMag,  filtYMag,  filtZMag  = 0,0,0

magXNormFiltered, magYNormFiltered, magZNormFiltered = 0,0,0

initialCompassCalib = 1

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

def calcPitchRollHeading(xAcc,yAcc,zAcc, xMag, yMag, zMag):
    normAcc = math.sqrt((xAcc*xAcc)+(yAcc*yAcc)+(zAcc*zAcc))
      
    sinRoll = -yAcc/normAcc
    cosRoll = math.sqrt(1.0-(sinRoll * sinRoll))
    sinPitch = xAcc/normAcc
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

    magXNormFilteredTilted = xMag*cosPitch+zMag*sinPitch
    magYNormFilteredTilted = xMag*sinRoll*sinPitch+yMag*cosRoll-zMag*sinRoll*cosPitch
        
    heading = (math.atan2(magYNormFilteredTilted,magXNormFilteredTilted)*180)/PI
    heading += 180
    
    if (heading < 0): heading = heading + 360;
        
    return roll, pitch, heading

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
    
    print("roll:%.1f pitch:%.1f heading:%.1f\r\n" % (roll,pitch,heading))

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
    
    