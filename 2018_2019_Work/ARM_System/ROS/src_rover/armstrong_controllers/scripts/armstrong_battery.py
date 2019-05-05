#!/usr/bin/env python

from __future__ import division
from numpy import pi, sqrt
from std_msgs.msg import String
import sys
import serial
import rospy

def talker():
    rospy.init_node('Armstrong_Battery', anonymous=True)
    while not rospy.is_shutdown():
        if(arduinoSerialData.inWaiting()>0):
            voltage = arduinoSerialData.readline()
            rospy.set_param("battery_level", str(voltage))

if __name__ == '__main__':
    arduinoSerialData = serial.Serial('/dev/ttyACM0',9600)
    talker()
