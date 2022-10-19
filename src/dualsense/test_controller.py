from pydualsense import pydualsense, TriggerModes

import time

# DualSense controller examples 
# https://flok.github.io/pydualsense/examples.html

def cross_pressed(state):
    print(state)

ds = pydualsense() # open controller
ds.init() # initialize controller

# RUMBLE THE MOTORS =====================================
print('Trigger Effect demo started')
ds.setLeftMotor(255)
time.sleep(2)
ds.setLeftMotor(0)
ds.setRightMotor(255)
time.sleep(2)
ds.setLeftMotor(0)
ds.setRightMotor(0)
# ======================================================

# Flash red 5 times ====================================
print('Flash red 5 times')
ds.light.setColorI(255, 0, 0)
time.sleep(1)
ds.light.setColorI(0, 0, 0)
time.sleep(1)
ds.light.setColorI(255, 0, 0)
time.sleep(1)
ds.light.setColorI(0, 0, 0)
time.sleep(1)
ds.light.setColorI(255, 0, 0)
time.sleep(1)
ds.light.setColorI(0, 0, 0)
# ======================================================


ds.cross_pressed += cross_pressed
ds.light.setColorI(0,0,0) # set touchpad color to red
# ds.triggerL.setMode(TriggerModes.Rigid)
# ds.triggerL.setForce(1, 255)

# close connection on control c interrupt
print("Cycling through colors")
print("Press control c to exit")
try:
    while True:
        # Slowly cycle through colors
        for i in range(0, 255):
            ds.light.setColorI(i, 0, 0)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255, i, 0)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255, 255, i)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255, 255, 255)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255, 255, 255 - i)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255, 255 - i, 0)
            time.sleep(0.01)
        for i in range(0, 255):
            ds.light.setColorI(255 - i, 0, 0)
            time.sleep(0.01)


except KeyboardInterrupt:
    ds.close()

ds.close() # closing the controller