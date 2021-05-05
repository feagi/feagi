import serial
import time
import struct

#ser = serial.Serial("/dev/ttyACM0", 9600)
ser = serial.Serial(
        port="/dev/ttyACM0",
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

while (1):
	#h1=ser.readline()
    if(ser.inWaiting()>0):
        value = ser.readline()
        #print(int('101',2))
        data=ser.readline().decode('ascii').strip(' ')
        print(int(data))
        print(type(int(data)))
    time.sleep(0.1)



