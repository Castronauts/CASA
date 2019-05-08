#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import xml
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
	pub_1 = rospy.Publisher('control_signal', Point, queue_size=10)
	pub_2 = rospy.Publisher('gripper_signal', Point, queue_size=10)
	pub_3 = rospy.Publisher('arlo_signal', Point, queue_size=10)
	kc, hz = start_node()
	rate = rospy.Rate(hz)
	
	while (not rospy.is_shutdown()):
		controls = kc.get_all_controls()
		control_msg = Point()
		gripper_msg = Point()
		arlo_msg = Point()
		control_msg.x = controls["x_axis"]["input"]
		control_msg.y = controls["y_axis"]["input"]
		control_msg.z = controls["z_axis"]["input"]
		gripper_msg.x = controls["gripper"]["input"] #Only using x for gripper signal. To lazy to make custom message and service
		arlo_msg.x = controls["fwd_bck"]["input"]
		arlo_msg.y = controls["left_right"]["input"]
		pub_1.publish(control_msg)
		pub_2.publish(gripper_msg)
		pub_3.publish(arlo_msg)
		rate.sleep()
		
	kc.stop_listener()
