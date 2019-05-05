#!/usr/bin/env python

'''
Service that maintains list of current obstacles to avoid for robot planning
'''



import rospy
import xml
import sys
from fetch_custom_msgs.srv import *

name_list = []
obstacle_list = []

def return_obstacles(req):
	num_obstacles = len(obstacle_list)
	flattened_obstacles = [i for subl in obstacle_list for i in subl]
	return ObstacleListResponse(num_obstacles, name_list, flattened_obstacles)

def list_obstacles(o_spec):
	global obstacle_list
	global name_list
	obstacle_list = []
	name_list = []
	for new_object in o_spec.iter('obstacle'):
		object_ = []
		obstacle_name = new_object.attrib['name']
		name_list.append(obstacle_name)
		object_.append(float(new_object.find('dim0_size').text))
		object_.append(float(new_object.find('dim1_size').text))
		object_.append(float(new_object.find('dim2_size').text))
		object_.append(float(new_object.find('orientation_w').text))
		object_.append(float(new_object.find('position_x').text))
		object_.append(float(new_object.find('position_y').text))
		object_.append(float(new_object.find('position_z').text))
		obstacle_list.append(object_)

def startup_server():
	rospy.init_node('get_obstacle_objects_server')
	myargv = rospy.myargv(argv=sys.argv)
	if(len(myargv) < 2):
		print("ERROR: XML file for obstacle spec not given")
		sys.exit(-1)
	obstacle_spec = xml.etree.ElementTree.parse(myargv[1])
	list_obstacles(obstacle_spec)
	s = rospy.Service('get_obstacle_objects', ObstacleList, return_obstacles)
	rospy.spin()

if __name__ == "__main__":
	startup_server()