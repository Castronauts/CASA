#! /usr/bin/env python

import rospy
#http://wiki.ros.org/sensor_msgs
from sensor_msgs.msg import JointState
#http://wiki.ros.org/dynamixel_msgs
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList
from math import pi
import time

rospy.init_node('Dynamixel_To_Move_Group') #Initialize node presence
rate = rospy.Rate(50) # 10hz

urdf_joints = ["Joint1", "Joint2", "Joint2", "Joint3", "Joint4", "Joint5", "Joint_Left", "Joint_Right"] #Joints corresponding to the IDs 1,2,3,4,5,6,7,8

dynamixel_radian_convert = [2048, 2048, 2048, 2048, 2048, 512, 734, 290] #Zero position offsets for radian conversion

MX_64_radian_convert = (360.0/4096)*(pi/180)
AX_12_18_radian_convert = (300.0/1023)*(pi/180)

#Subscriber function in order to get the current states
def joint_state_topic_initialize(msg):
	joint_states = JointState()

	joint_states.header.stamp = rospy.Time.now()

	for motor in msg.motor_states:
		joint_states.name.append(urdf_joints[motor.id-1])

		if(motor.id == 6): 
			joint_states.position.append((motor.position - dynamixel_radian_convert[motor.id-1]) * AX_12_18_radian_convert)
		elif(motor.id > 6):
			joint_states.position.append((dynamixel_radian_convert[motor.id-1] - motor.position) * AX_12_18_radian_convert)
			print("HAHAHAHAHAHAHAHA")
			print(motor.id)
			print(motor.position)
			print("HAHAHAHAHAHAHAHA")
		else:
			joint_states.position.append((motor.position - dynamixel_radian_convert[motor.id-1]) * MX_64_radian_convert)

	publisher.publish(joint_states)

subscriber = rospy.Subscriber('/motor_states/dynamixel_port', MotorStateList, joint_state_topic_initialize) #Raw feedback subscriber get the information in order to publish

publisher = rospy.Publisher('joint_states', JointState, queue_size=10) #Publisher for publishing the joint states

rospy.spin() #Spin until close out