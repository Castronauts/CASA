#!/usr/bin/env python

from __future__ import division
from numpy import pi, sqrt
from std_msgs.msg import String
import sys
import serial
import rospy

def callback(message):
	arduino_serial.write(message.data)
	
def listener():
	rospy.init_node('Motor_Control_Listener', anonymous=True)
	
	rospy.Subscriber("arlo_wheels", String, callback)
	
	rospy.spin()


if __name__ == '__main__':
	arduino_serial = serial.Serial("/dev/ttyACM0", 9600)
	listener()
