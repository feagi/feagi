import serial
import time

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
    print(int(ser.readline()))
    time.sleep(0.1)
