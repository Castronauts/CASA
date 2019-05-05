import serial

ser = serial.Serial('/dev/ttyUSB0',timeout=1)

while(1):
    ser.write("4084")
    ser.write(" ")
    ser.write("4084")
    ser.write("\n")

    response = ser.readline()
