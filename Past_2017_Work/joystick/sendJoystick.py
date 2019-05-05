from __future__ import print_function
import serial
import sys
# import os

# f = open('logSend.log', 'w')
ser = serial.Serial('/dev/ttyUSB0', timeout=1)
# os.system('clear')

while(1):
	for line in sys.stdin:
		# f.write(line)
		# printing to stdout shows the pipe functioning perfectly; however,
		# when sending through serial some unknown problem occurs
		# ser.write(line)
		sys.stdout.write(line)

		# the response is as so: xangle yangle latdeg latmin latfracmin latfracmin latdir longdeg longmin
		# longfracmin longfracmin longdir altitude time \n
		# response = ser.readline()
		# reslist = response.split(" ")
		# if (reslist != [""]):
			# pitch = reslist[0]
			# roll = reslist[1].strip()
			# print("Pitch: " + str('%g' % round(float(pitch))) + u'\u00b0' + "                    ", end='\n')
			# print("Roll: " + str('%g' % round(float(roll))) + u'\u00b0' + "                  ", end='\033[1A' + '\r')
			# sys.stdout.flush()
