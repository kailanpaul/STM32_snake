import serial

ser = serial.Serial('/dev/ttyACM0')
ser.write(b'x')

while True:
    print(ser.read())