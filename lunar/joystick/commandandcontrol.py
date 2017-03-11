import serial
import pygame

def convert_range( raw ):
    return ((raw*2042)+2042) #this is an artifact of how we sent values with the analog joystick, easiest to just not change 

pygame.init()
pygame.joystick.init()
cmdjoystick = pygame.joystick.Joystick(0)
cmdjoystick.init()
camjoystick = pygame.joystick.Joystick(1)
camjoystick.init()
ser = serial.Serial('/dev/ttyUSB0',timeout=1)

while(1):
        event = pygame.event.poll() #I don't totally understand the inner workings of pygame, but this is necessary to update joystick state
        #can add delay here
        #print(str(convert_range(cmdjoystick.get_axis(1))))
        #print(str(convert_range(cmdjoystick.get_axis(0))))
        ser.write(str(convert_range(-cmdjoystick.get_axis(1)))) #this is negative because this and the analog joystick have opposite directions on that one axis
        ser.write(" ")
        ser.write(str(convert_range(cmdjoystick.get_axis(0))))
        ser.write("\n")

        response = ser.readline()
