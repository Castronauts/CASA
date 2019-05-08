#!/usr/bin/env python

import rospy
import xml
import sys
import KeyboardController
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

def start_node():
	myargv = rospy.myargv(argv=sys.argv)
	if(len(myargv) < 3):
		print("ERROR: XML files for controller spec not given")
		sys.exit(-1)
	controller_spec = xml.etree.ElementTree.parse(myargv[1])
	keyboard_spec = xml.etree.ElementTree.parse(myargv[2])
	kc = KeyboardController.DiscreteKeyboardController(control_spec, keyboard_spec)
	kc.start_listener()
	return kc


if __name__ == '__main__':
    rospy.init_node("Robot_Controller")
    kc = start_node()
    #TODO: publish controls on a regular basis