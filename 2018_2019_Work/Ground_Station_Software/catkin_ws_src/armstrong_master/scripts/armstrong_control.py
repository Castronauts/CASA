#!/usr/bin/env python

from __future__ import division
import sys

from numpy import pi, sqrt
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Float64, String, Header
from dynamixel_msgs.msg import JointState
from actionlib_msgs.msg import GoalStatusArray

class ArmstrongMaster:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('armstrong_control')
        self.robot = moveit_commander.RobotCommander()
        self.arm = ArmstrongInterface('Arm')
        self.gripper = ArmstrongGripper()
        self.state = rospy.Publisher("/end_effector_pose", PoseStamped, queue_size=20)
        
        #Move the arm to the Start_Arm position being the base position
        self.reset()

    def reset(self):
        self.arm.reset()
    	self.gripper.open_full()
        rospy.sleep(3) #Wait for 3 second
        
    def main_loop(self):
        rate = rospy.Rate(10)
        sequence = 1

        while not rospy.is_shutdown():

            if (sequence < 10):
            	self.arm.grab_stand_sequence(sequence)
                self.gripper.grab_stand_sequence(sequence)
                rospy.sleep(1)
                sequence = sequence + 1

            pose = self.arm.get_current_pose()
            self.state.publish(pose)
            rate.sleep()

class ArmstrongInterface(moveit_commander.MoveGroupCommander):
    def __init__(self, name):
    	moveit_commander.MoveGroupCommander.__init__(self, name) #Initialize parent class first
    	self.set_end_effector_link("Tip_Frame")
    	self.set_pose_reference_frame("/base_link")
    	self.set_planning_time(10) #Set planning time for 10 seconds
    	self.set_num_planning_attempts(20)
    	self.set_goal_tolerance(0.01)

    def reset(self):
    	self.set_named_target("Arm_Start") #Specify initially declared joint value poses from MoveIt Setup
    	self.go() #Set target group then move to specified target

    def grab_stand_sequence(self, number):
        if (number == 0):
            self.set_named_target("Arm_Start")
            self.go()
        elif (number == 1):
            self.set_named_target("Video_Stand_1")
            self.go()
        elif (number == 2):
            self.set_named_target("Video_Stand_2")
            self.go()
        elif (number == 3):
            self.set_named_target("Video_Stand_3")
            self.go()
        elif (number == 4):
            self.set_named_target("Video_Stand_4")
            self.go()
        elif (number == 5):
            self.set_named_target("Video_Stand_5")
            self.go()
        elif (number == 6):
            self.set_named_target("Video_Stand_6")
            self.go()
        elif (number == 7):
            self.set_named_target("Video_Stand_7")
            self.go()
        elif (number == 8):
            self.set_named_target("Video_Stand_8")
            self.go()
        elif (number == 9):
            self.set_named_target("Video_Stand_9")
            self.go()

    
        
class ArmstrongGripper:
    def __init__(self):
    	gripper_topic = rospy.get_param("~gripper_topic", "/dual_gripper_controller/command") #Create private gripper topic for easy open and close implementation
    	self.gripper_command = rospy.Publisher(gripper_topic, Float64, queue_size=10)

    def open_full(self):
    	self.gripper_command.publish(-1.5)

    def close_full(self):
        self.gripper_command.publish(0.0)
    
    '''Send values continously to open and close accordingly'''
    def close_control(self, value):
        self.gripper_command.publish(value)

    def open_control(self, value):
        self.gripper_command.publish(value)

    def grab_stand_sequence(self, number):
        if (number == 0):
            self.gripper_command.publish(-1.5)
        elif (number == 1):
            self.gripper_command.publish(-1.5)
        elif (number == 2):
            self.gripper_command.publish(-0.3)
        elif (number == 3):
            self.gripper_command.publish(-0.13)
        elif (number == 4):
            self.gripper_command.publish(-0.13)
        elif (number == 5):
            self.gripper_command.publish(-0.13)
        elif (number == 6):
            self.gripper_command.publish(-0.13)
        elif (number == 7):
            self.gripper_command.publish(-0.13)
        elif (number == 8):
            self.gripper_command.publish(-0.3)
        elif (number == 9):
            self.gripper_command.publish(-0.3)

if __name__ == "__main__":
	ArmstrongMaster().main_loop() #Run main loop which runs the pick and place sequence once command is initiated