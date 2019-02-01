import readArduino as ra
import signal
import sys
import numpy as np
from numpy.random import randn

class Read:

    def run(self, sensorInstance):
        try:
            running = True
            while running:
                data = sensorInstance.getSerialData()
                imu = np.array([[data[0],data[1],data[2]],[data[3],data[4],data[5]],[data[6],data[7],data[7]]])
                print(imu)

        except KeyboardInterrupt:
            sensorInstance.close()
            print('interrupted!')
            sys.exit(0)
            


portName = 'COM30'
baudRate = 115200
dataNumBytes = 2  
numParams = 9  
s = ra.SerialRead(portName, baudRate, dataNumBytes, numParams)  
s.readSerialStart()  
pv = Read()
pv.run(s)