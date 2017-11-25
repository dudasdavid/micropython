#!/usr/bin/env python
"""This script shows another example of using the PyWavefront module."""
# This example was created by intrepid94
import sys
sys.path.append('..')
import ctypes
import pyglet
from pyglet.gl import *
from pywavefront import Wavefront
import serial
import serial.tools.list_ports

print "Available serial devices:"

serialList = serial.tools.list_ports.comports()
ports = []
i = 0
for element in serialList:
    i+=1
    print("%d: %s" % (i, element[0]))
    ports.append(element[0])
    
devIndex = raw_input("\nSelect device: ")    
selectedDevice = ports[int(devIndex)-1]

print("Selected device is %s." % selectedDevice)

ser = serial.Serial(
    port=selectedDevice,
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.close()
ser.open()
ser.isOpen()

out = ''   
roll = 0
pitch = 0
heading = 0

meshes = Wavefront('stm32f411.obj')

window = pyglet.window.Window(800, 600, caption = 'Demo', resizable = False)
glClearColor(0.8,0.9,1,1)

lightfv = ctypes.c_float * 4
# label = pyglet.text.Label('Hello, world', font_name = 'Times New Roman', font_size = 12, x = 100, y = 100, anchor_x = 'center', anchor_y = 'center')
@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(40.0, float(width)/height, 1, 100.0)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_MODELVIEW)
    return True

@window.event
def on_draw():
    window.clear()
    glLoadIdentity()
    # glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-40, 200, 100, 0.0))
    # glLightfv(GL_LIGHT0, GL_AMBIENT, lightfv(0.2, 0.2, 0.2, 1.0))
    glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-1.0, 1.0, 1.0, 0.0))
    # glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 1.0))
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHTING)
    # glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH)
    glMatrixMode(GL_MODELVIEW)
    # glTranslated(0, 4, -8)
 #    glRotatef(90, 0, 1, 0)
 #    glRotatef(-60, 0, 0, 1)
   # Rotations for sphere on axis - useful
    glTranslated(0, 0, -4)
    glRotatef(0, 0, 1, 0)
    glRotatef(heading, 0, 1, 0)
    glRotatef(roll, 1, 0, 0)
    glRotatef(pitch, 0, 0, 1)
    
    # glRotatef(90, 0, 0, 1)
    # glRotatef(0, 0, 1, 0)
    meshes.draw()
def update(dt):
    global out
    global roll, pitch, heading
    out = ''
    while ser.inWaiting() > 0:
        out += ser.read(1)
            
    if out != '':
        print out
        rawData = out.split("\r\n")[0].split(" ")
        print rawData
        try:
            roll = float(rawData[0].split("roll:")[1])
            pitch = float(rawData[1].split("pitch:")[1])*-1
            heading = 360-float(rawData[2].split("heading:")[1])

            print roll, pitch, heading
    
        except:
            print "Invalid data was received!"


pyglet.clock.schedule(update)

pyglet.app.run()
