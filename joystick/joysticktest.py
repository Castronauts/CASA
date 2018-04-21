import pygame

pygame.init()
pygame.joystick.init()

while(1):
	joystick = pygame.joystick.Joystick(0)
	joystick.init()

	event = pygame.event.poll()
	# Note, we need to scale these to be the size the rover expects, map the range to the rover range
	print(joystick.get_axis(0), joystick.get_axis(1), joystick.get_button(0), joystick.get_button(1), joystick.get_button(2), joystick.get_button(3))
	# right now this reads joystsick input as the joystick is moved.
	# The Arduino code spins on the last command until it receives a new one.
	# This makes sense for the joystick, though we'll have to see how it does outside
	# (losing signal and driving off seems like a problem)
	# Also, we probably need to maintain synchronicity of commands still
	# note, we need to do the above step to get it to process the joystick events,
	# even though we're not doing anything with them
	# NOTE, for pygame to work we need to initialize the display, even if we don't use it,
	# since its involved in ordering events
	# we will likely want to ignore the queue, but we may have to use pygame.event.pump()
	# to syncrhonize the game with the system (I don't totally understand this)
