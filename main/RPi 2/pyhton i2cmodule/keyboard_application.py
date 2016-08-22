import curses
import i2cmodule as i2c
from time import sleep

mecanumBaseAddr = 0x20

i2c.initI2C()

i2c.openPort(mecanumBaseAddr)

i2c.jointReset()

i2c.baseReset()

i2c.setControlMode(1)

i2c.baseSetSpeed(0.0, 0.0, 0.10)

"""
while(1):
	jointPos = i2c.baseGetSpeed()
	if jointPos is not None:
		print(jointPos[0], jointPos[1], jointPos[2])
	sleep(0.05)
"""