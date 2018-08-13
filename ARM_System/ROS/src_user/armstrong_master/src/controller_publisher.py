#!/usr/bin/env python

import rospy
import xml
import sys
import KeyboardController
from geometry_msgs.msg import Point

def start_node():
	myargv = rospy.myargv(argv=sys.argv)
	if(len(myargv) < 4):
		print("ERROR: XML files for controller spec not given")
		sys.exit(-1)
	controller_spec = xml.etree.ElementTree.parse(myargv[1])
	keyboard_spec = xml.etree.ElementTree.parse(myargv[2])
	kc = KeyboardController.DiscreteKeyboardController(controller_spec, keyboard_spec)
	kc.start_listener()
	return kc, int(myargv[3])


if __name__ == '__main__':
    rospy.init_node("Robot_Controller")
    pub = rospy.Publisher('control_signal',	Point, queue_size=1)
    kc, hz = start_node()
    rate = rospy.Rate(hz)
    while (not rospy.is_shutdown()):
    	controls = kc.get_all_controls()
    	control_msg = Point()
    	control_msg.x = controls["x_axis"]["input"]
    	control_msg.y = controls["y_axis"]["input"]
    	control_msg.z = controls["z_axis"]["input"]
    	pub.publish(control_msg)
    	rate.sleep()
    kc.stop_listener()