from __future__ import print_function
import serial
import pygame
import time
import sys
import os



def convert_range(raw):
	# this is an artifact of how we sent values with the analog joystick, easiest to just not change
	return ((raw * 2042) + 2042)

pygame.init()
pygame.joystick.init()
cmdjoystick = pygame.joystick.Joystick(0)
cmdjoystick.init()
camjoystick = pygame.joystick.Joystick(1)
camjoystick.init()
ser = serial.Serial('/dev/ttyUSB0', timeout=1)
os.system('clear')

while(1):
	# I don't totally understand the inner workings of pygame, but this is necessary to update joystick state
	event = pygame.event.poll()
	# can add delay here
	time.sleep(1.3)
	print(str(convert_range(cmdjoystick.get_axis(1))))
	print(str(convert_range(cmdjoystick.get_axis(0))))
	print(str(convert_range(camjoystick.get_axis(0))))
	print(str(convert_range(camjoystick.get_axis(0))))
	if (camjoystick.get_button(1) == 1):
		ser.write("c 0 0 0\n")  # center camera command

	# this is negative because this and the analog joystick have opposite directions on that one axis
	ser.write(str(convert_range(cmdjoystick.get_axis(1))))
	ser.write(" ")
	ser.write(str(convert_range(cmdjoystick.get_axis(0))))
	ser.write(" ")
	ser.write(str(convert_range(-camjoystick.get_axis(1))))
	ser.write(" ")
	ser.write(str(convert_range(camjoystick.get_axis(0))))
	ser.write("\n")

	response = ser.readline()
	# the response is as so: xangle yangle latdeg latmin latfracmin latfracmin latdir longdeg longmin
	# longfracmin longfracmin longdir altitude time \n
	reslist = response.split(" ")
	if (reslist != [""]):
		pitch = reslist[0]
		roll = reslist[1].strip()
		print("Pitch: " + str('%g' % round(float(pitch))) + u'\u00b0' + "                    ", end='\n')
		print("Roll: " + str('%g' % round(float(roll))) + u'\u00b0' + "                  ", end='\033[1A' + '\r')
		sys.stdout.flush()
