import serial

ser = serial.Serial('/dev/ttyUSB0', timeout=1)

print('Welcome to Arlo control')
