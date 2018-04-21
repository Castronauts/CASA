from __future__ import print_function
import pyuarm
import pygame
import os
import time


pygame.init()
pygame.joystick.init()

cmdjoystick = pygame.joystick.Joystick(0)
cmdjoystick.init()

arm = pyuarm.get_uarm()
s = 5
suction = False
gripper = False
time.sleep(5)

angle0 = arm.get_servo_angle(0)
angle1 = arm.get_servo_angle(1)
angle2 = arm.get_servo_angle(2)
angle3 = arm.get_servo_angle(3)

os.system('clear')

while(1):
	pygame.event.pump()

	if abs(cmdjoystick.get_axis(2)) > 0.1:
		angle0 = angle0 - cmdjoystick.get_axis(2) * s
		# print("New Angle 0: " + str(arm.get_servo_angle(0)) + ' + ' + str(cmdjoystick.get_axis(2) * s) + ' = ' + str(newAngle) + '\r')
		if angle0 < 0:
			angle0 = 0
		elif 180 < angle0:
			angle0 = 180
		arm.set_servo_angle(0, angle0)

	if abs(cmdjoystick.get_axis(3)) > 0.1:
		angle1 = angle1 + cmdjoystick.get_axis(3) * s
		# print("New Angle 0: " + str(arm.get_servo_angle(1)) + ' + ' + str(cmdjoystick.get_axis(3) * s) + ' = ' + str(newAngle) + '\r')
		if angle1 < 0:
			angle1 = 0
		elif 180 < angle1:
			angle1 = 180
		arm.set_servo_angle(1, angle1)

	if cmdjoystick.get_button(9) == 1:
		angle2 = angle2 + s
		# print("New Angle 0: " + str(arm.get_servo_angle(2)) + ' + ' + str(s) + ' = ' + str(newAngle) + '\r')
		if 180 < angle2:
			angle2 = 180
		arm.set_servo_angle(2, angle2)

	if cmdjoystick.get_button(11) == 1:
		angle2 = angle2 - s
		if angle2 < 0:
			angle2 = 0
		# print("New Angle 0: " + str(arm.get_servo_angle(2)) + ' - ' + str(s) + ' = ' + str(newAngle) + '\r')
		arm.set_servo_angle(2, angle2)

	if cmdjoystick.get_button(7) == 1:
		angle3 = angle3 + s
		if 180 < angle3:
			angle3 = 180
		arm.set_servo_angle(3, angle3)

	if cmdjoystick.get_button(5) == 1:
		angle3 = angle3 - s
		if angle3 < 0:
			angle3 = 0
		arm.set_servo_angle(3, angle3)

	if cmdjoystick.get_button(2) == 1:
		suction = not suction
		gripper = not gripper
		arm.set_gripper(gripper)
		arm.set_pump(suction)


	print("Servo 0 angle: " + str(arm.get_servo_angle(0)))
	print("Servo 1 angle: " + str(arm.get_servo_angle(1)))
	print("Servo 2 angle: " + str(arm.get_servo_angle(2)))
	print('\033[4A', end='')
