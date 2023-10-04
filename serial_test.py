import serial
import os
import sys
import glob
import serial.tools.list_ports

print([comport.device for comport in serial.tools.list_ports.comports()][0])

if os.name == 'nt':
    ser = serial.Serial('COM5', 9600)
else:
    ser = serial.Serial('/dev/ttyACM0')

ser.write(b'x')

while True:
    print(ser.read())