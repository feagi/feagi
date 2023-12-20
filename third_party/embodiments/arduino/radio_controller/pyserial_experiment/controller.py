import time
import json
import serial

if __name__ == "__main__":
    print ("Ready...")
    ser  = serial.Serial("COM5", baudrate= 9600, 
           timeout=2.5, 
           parity=serial.PARITY_NONE, 
           bytesize=serial.EIGHTBITS, 
           stopbits=serial.STOPBITS_ONE
        )
    data = dict()
    data["2"] = 1

    data=json.dumps(data)
    print (data)
    if ser.isOpen():
        ser.write(data.encode('ascii'))
        time.sleep(1)
        ser.readline()
        time.sleep(2)
    print("done")