import serial


ser = serial.Serial("/dev/ttyACM0", 9600)
while True:
    input_value = input('Enter input from 0-255: ')
    ser.write(input_value.encode()) #if you put more than 255, it wouldn't bypass the 255 due to configuration on H-brighe
