from __future__ import print_function
import serial
import pygame
import sys
import os
import csv


def convert_range( raw ):
    return ((raw*2042)+2042) #this is an artifact of how we sent values with the analog joystick, easiest to just not change 

pygame.init()
pygame.joystick.init()
cmdjoystick = pygame.joystick.Joystick(0)
cmdjoystick.init()
camjoystick = pygame.joystick.Joystick(1)
camjoystick.init()
ser = serial.Serial('/dev/ttyUSB0',timeout=1)
os.system('clear')
csvfile = open(sys.argv[1],'wb')

while(1):
        event = pygame.event.poll() #I don't totally understand the inner workings of pygame, but this is necessary to update joystick state
        #can add delay here
        #print(str(convert_range(cmdjoystick.get_axis(1))))
        #print(str(convert_range(cmdjoystick.get_axis(0))))
        if ( camjoystick.get_button(1) == 1):
            ser.write("c 0 0 0\n") #center camera command
        ser.write(str(convert_range(cmdjoystick.get_axis(1)))) #this is negative because this and the analog joystick have opposite directions on that one axis
        ser.write(" ")
        ser.write(str(convert_range(cmdjoystick.get_axis(0))))
        ser.write(" ")
        ser.write(str(convert_range(-camjoystick.get_axis(1))))
        ser.write(" ")
        ser.write(str(convert_range(camjoystick.get_axis(0))))
        ser.write("\n")

        response = ser.readline()
        reslist = response.split(" ") #the response is as so: xangle yangle latdeg latmin latfracmin latfracmin latdir longdeg longmin longfracmin longfracmin longdir altitude time \n
        print(reslist) 
        if (reslist != [""]):
            pitch = reslist[0]
            roll = reslist[1]
            print("Pitch: " + str('%g'%round(float(pitch))) + u'\u00b0' + "                    ", end='\n')
            print("Roll: " + str('%g'%round(float(roll))) + u'\u00b0' + "                  ", end = '\033[1A' + '\r')
            mywriter = csv.writer(csvfile, delimiter=',',quotechar="'", quoting=csv.QUOTE_MINIMAL)
            mywriter.writerow([reslist[2],reslist[3],reslist[4],reslist[5],reslist[6],reslist[7],reslist[8],reslist[9],reslist[10],reslist[11],reslist[12]])
            sys.stdout.flush()
            csvfile.flush()