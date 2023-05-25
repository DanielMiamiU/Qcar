from Quanser.q_ui import gamepadViaTarget
from Quanser.product_QCar import QCar
import time
import numpy as np
import os
import struct


sampleRate = 1000
sampleTime = 1/sampleRate

## Gamepad Initialization
gpad = gamepadViaTarget(1) 


counter = 0

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## QCar Initialization
myCar = QCar()

new = gpad.read()
try:
    while gpad.B != 1:
        mtr_cmd = np.array([.2*gpad.RT, gpad.left - gpad.right])
        LEDs = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        new = gpad.read()

        os.system('clear')
		
        print("Right Trigger:\t\t{0:.2f}\nleft:\t\t\t{1:.0f}\nRight:\t\t\t{2:.0f}\nMotor Throttle:\t\t\t{3:4.2f}".format(gpad.RT, gpad.left, gpad.right, mtr_cmd[0]))

        current, batteryVoltage, encoderCounts = myCar.read_write_std(mtr_cmd, LEDs)   

except KeyboardInterrupt:
	print("User interrupted!")

gpad.terminate()
myCar.terminate()