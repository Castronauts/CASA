import pygame
import sys
# import os


def convert_range(raw):
	# this is an artifact of how we sent values with the analog joystick, easiest to just not change
	return ((raw * 2042) + 2042)

pygame.init()
pygame.joystick.init()
cmdjoystick = pygame.joystick.Joystick(0)
cmdjoystick.init()
camjoystick = pygame.joystick.Joystick(1)
camjoystick.init()
# os.system('clear')

# f = open('logRead.log', 'w')

while(1):
	event = pygame.event.poll()

	if (camjoystick.get_button(1) == 1):
		sys.stdout.write("c 0 0 0\n")  # center camera command

	# f.write(str(convert_range(cmdjoystick.get_axis(1))))
	# f.write(' ')
	# f.write(str(convert_range(cmdjoystick.get_axis(0))))
	# f.write(' ')
	# f.write(str(convert_range(-camjoystick.get_axis(1))))
	# f.write(' ')
	# f.write(str(convert_range(camjoystick.get_axis(0))))
	# f.write('\n')

	sys.stdout.write(str(convert_range(cmdjoystick.get_axis(1))))
	sys.stdout.write(' ')
	sys.stdout.write(str(convert_range(cmdjoystick.get_axis(0))))
	sys.stdout.write(' ')
	# this is negative because this and the analog joystick have opposite directions on that one axis
	sys.stdout.write(str(convert_range(-camjoystick.get_axis(1))))
	sys.stdout.write(' ')
	sys.stdout.write(str(convert_range(camjoystick.get_axis(0))))
	sys.stdout.write('\n')
