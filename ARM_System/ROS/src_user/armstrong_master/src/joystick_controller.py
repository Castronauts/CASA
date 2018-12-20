#!/usr/bin/env python
# -*- coding: utf-8 -*-

#Standard libraries
import sys
import pygame
import time
from tcpcom import TCPClient #IP socket for camera
import math

#ROS Libraries
import rospy
from std_msgs.msg import Float64, String, Int64
from geometry_msgs.msg import Point
from dynamixel_msgs.msg import JointState

#Xbox libraries
from xbox360controller import Xbox360Controller

class Joystick_Object(object):
    def __init__(self):
        #Extra Variables
        self.gripper_load = 0.0

        #----------------------------------------------------------------------------------------------------
        #Pan/Tilt Camera IP Socket
        #----------------------------------------------------------------------------------------------------
        self.IP_ADDRESS = "192.168.1.9"
        self.IP_PORT = 23000
        self.client = TCPClient(self.IP_ADDRESS, self.IP_PORT, stateChanged = self.onStateChanged)
        self.rc = self.client.connect()

        if(self.rc):
            print("Connected")
        else:
            print("Not connected")

        #----------------------------------------------------------------------------------------------------
        #Pygame/Joystick Initialization
        #----------------------------------------------------------------------------------------------------
        pygame.init()

        # Initialize the joysticks
        pygame.joystick.init()

        #Initialize only one joystick connected
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        #Initiate Xbox Controller
        self.xbox = Xbox360Controller()

        #----------------------------------------------------------------------------------------------------
        #ROS Initialization
        #----------------------------------------------------------------------------------------------------
        rospy.init_node("joystick_controller")

        #Initialize arm stuff
        self.arm_pub = rospy.Publisher('control_signal', Point, queue_size=10)
        self.arm_msg = Point()

        rospy.Subscriber("xbox_vibrate", Int64, self.vibrateXbox)

        #Initialize gripper stuff
        self.gripper_pub = rospy.Publisher("/dual_gripper_controller/command", Float64, queue_size=10)
        rospy.Subscriber("/dual_gripper_controller/state", JointState, self.updateGripperLoad)
        self.gripper_msg = Float64()
        self.gripper_msg.data = 0.0
        self.gripper_pub.publish(self.gripper_msg)

        #Initialize rover stuff
        self.rover_pub = rospy.Publisher("arlo_wheels", String, queue_size=10)
        self.rover_msg = String()
        self.rover_speed = 40.0 #Out of 200 max
        self.rover_msg.data = "rst\r"
        self.rover_pub.publish(self.rover_msg)

        #Rate for loop
        self.rate = rospy.Rate(10) #10 Hertz

    #----------------------------------------------------------------------------------------------------
    #Function Declarations
    #----------------------------------------------------------------------------------------------------
    def onStateChanged(self, state, msg):
        if state == "CONNECTING":
           print "Client:-- Waiting for connection..."
        elif state == "CONNECTED":
           print "Client:-- Connection estabished."
        elif state == "DISCONNECTED":
           print "Client:-- Connection lost."

    def updateGripperLoad(self, msg):
        self.gripper_load = msg.load

    def vibrateXbox(self, msg):
        if (msg.data == 1):
            self.xbox.set_rumble(0.5, 0.5, 500)
            rospy.sleep(0.5)

    def degreeHeading(self, x, y):
        #Convert to degree heading
        heading_degree = math.atan2((x), (y*-1.0)) #Flipped orientation

        if (heading_degree < 0.0):
            heading_degree += 2.0 * math.pi

        heading_degree *= 180.0 / math.pi

        final_degree = int(heading_degree)

        #Need 360 degree values 0...359
        if(final_degree > 359):
            final_degree = 359

        return final_degree

    def runLoop(self):
        while (not rospy.is_shutdown()):
            #Check user input
            pygame.event.get()

            #Get Joystick values
            x_left = self.joystick.get_axis(0)
            y_left = self.joystick.get_axis(1)
            trigger_left = self.joystick.get_axis(2)
            x_right = self.joystick.get_axis(3)
            y_right = self.joystick.get_axis(4)
            trigger_right = self.joystick.get_axis(5)

            #Get Hat Values
            camera_pads = self.joystick.get_hat(0) #[0]-left,right [1]-top,down

            #Get Button Values
            open_button = self.joystick.get_button(0) #A button
            close_button = self.joystick.get_button(1) #B button

            #Get bumper values
            arm_down = self.joystick.get_button(4) #Left bumper
            arm_up = self.joystick.get_button(5) #Right bumper

            #----------------------------------------------------------------------------------------------------
            #Arm Commands
            #----------------------------------------------------------------------------------------------------
            #Check height arm buttons
            if (arm_up != 0.0 or arm_down != 0.0):

                #Check only up
                while(arm_up != 0.0 and arm_down == 0.0):

                    #Check thumbstick control
                    if(abs(x_right) >= 0.6 or abs(y_right) >= 0.6):
                        print("Up with thumbstick")

                    #Just up
                    else:
                        self.arm_msg.z = 1.0
                        self.arm_pub.publish(self.arm_msg)

                    #Recheck values
                    pygame.event.get()
                    arm_down = self.joystick.get_button(4)
                    arm_up = self.joystick.get_button(5)
                    rospy.sleep(0.1)

                #Check only down
                while(arm_down != 0.0 and arm_up == 0.0):

                    #Check thumbstick control
                    if(abs(x_right) >= 0.6 or abs(y_right) >= 0.6):
                        print("Down with thumbstick")

                    #Just down
                    else:
                        self.arm_msg.z = -1.0
                        self.arm_pub.publish(self.arm_msg)

                    #Recheck values
                    pygame.event.get()
                    arm_down = self.joystick.get_button(4)
                    arm_up = self.joystick.get_button(5)
                    rospy.sleep(0.1)

                #Turn off motion
                self.arm_msg.z = 0.0
                self.arm_msg.y = 0.0
                self.arm_msg.x = 0.0
                self.arm_pub.publish(self.arm_msg)
                rospy.sleep(0.1)

            #Check only joystick arm x and y
            if ((abs(x_right) >= 0.6 or abs(y_right) >= 0.6) and (arm_up == 0.0 and arm_down == 0.0)):

                while ((abs(x_right) >= 0.6 or abs(y_right) >= 0.6)):

                    #Break if bumpers are pressed to go into that mode
                    if(arm_up != 0.0 or arm_down != 0.0):
                        self.arm_msg.z = 0.0
                        self.arm_msg.y = 0.0
                        self.arm_msg.x = 0.0
                        self.arm_pub.publish(self.arm_msg)
                        rospy.sleep(0.1)
                        break

                    #Get heading
                    final_degree = self.degreeHeading(x_right, y_right)

                    #Check forward signal
                    if ((final_degree > 315 and final_degree <=359) or (final_degree >= 0 and final_degree <= 45)):
                        self.arm_msg.z = 0.0
                        self.arm_msg.y = 1.0
                        self.arm_msg.x = 0.0
                        self.arm_pub.publish(self.arm_msg)

                    #Check right signal
                    elif (final_degree > 45 and final_degree <= 135):
                        self.arm_msg.z = 0.0
                        self.arm_msg.y = 0.0
                        self.arm_msg.x = 1.0
                        self.arm_pub.publish(self.arm_msg)

                    #Check backward signal
                    elif (final_degree > 135 and final_degree <= 225):
                        self.arm_msg.z = 0.0
                        self.arm_msg.y = -1.0
                        self.arm_msg.x = 0.0
                        self.arm_pub.publish(self.arm_msg)

                    #Check left signal
                    elif (final_degree > 225 and final_degree <= 315):
                        self.arm_msg.z = 0.0
                        self.arm_msg.y = 0.0
                        self.arm_msg.x = -1.0
                        self.arm_pub.publish(self.arm_msg)

                    #Recheck values
                    pygame.event.get()
                    arm_down = self.joystick.get_button(4)
                    arm_up = self.joystick.get_button(5)
                    x_right = self.joystick.get_axis(3)
                    y_right = self.joystick.get_axis(4)

                    rospy.sleep(0.1)

                #Shutdown values
                self.arm_msg.z = 0.0
                self.arm_msg.y = 0.0
                self.arm_msg.x = 0.0
                self.arm_pub.publish(self.arm_msg)
                rospy.sleep(0.1)


            #----------------------------------------------------------------------------------------------------
            #Gripper Commands
            #----------------------------------------------------------------------------------------------------
            #Check open close buttons
            if (open_button != 0.0 or close_button != 0.0):
                #Check open button only
                while (open_button != 0 and close_button == 0.0 and self.gripper_msg.data >= -0.4):
                    #Send and sleep to repeat
                    self.gripper_msg.data = self.gripper_msg.data - 0.01
                    self.gripper_pub.publish(self.gripper_msg)
                    rospy.sleep(0.02)

                    #Recheck values
                    pygame.event.get()
                    open_button = self.joystick.get_button(0)

                #Check close button only
                while (close_button != 0 and open_button == 0.0 and self.gripper_msg.data <= 0.4 and self.gripper_load > -0.1):
                    #Send and sleep to repeat
                    self.gripper_msg.data = self.gripper_msg.data + 0.01
                    self.gripper_pub.publish(self.gripper_msg)
                    rospy.sleep(0.02)

                    #Recheck values
                    pygame.event.get()
                    close_button = self.joystick.get_button(1)

            #----------------------------------------------------------------------------------------------------
            #Rover Commands
            #----------------------------------------------------------------------------------------------------
            #Check triggers
            if (trigger_left >= 0.5 or trigger_right >= 0.5):

                #Check forward command only
                while (trigger_right >= 0.5 and trigger_left <= 0.0):

                    #Check thumbstick control
                    #Account for forward heading orientation
                    if(abs(x_left) >= 0.6 or abs(y_left) >= 0.6):

                        #Get heading
                        final_degree = self.degreeHeading(x_left, y_left)

                        #Check left right signal
                        if (final_degree > 180 and final_degree <= 359):
                            self.rover_msg.data = "gospd " + str(int(self.rover_speed * 0.1)) + " " + str(int(self.rover_speed)) + "\r" #Only takes integers
                            self.rover_pub.publish(self.rover_msg)
                        else:
                            self.rover_msg.data = "gospd " + str(int(self.rover_speed)) + " " + str(int(self.rover_speed * 0.1)) + "\r" #Only takes integers
                            self.rover_pub.publish(self.rover_msg)

                        rospy.sleep(0.1)

                    #Just forward
                    else:
                        self.rover_msg.data = "gospd " + str(int(self.rover_speed)) + " " + str(int(self.rover_speed)) + "\r" #Only takes integers
                        self.rover_pub.publish(self.rover_msg)
                        rospy.sleep(0.1)

                    #Recheck values neglecting left trigger
                    pygame.event.get()
                    trigger_right = self.joystick.get_axis(5)
                    x_left = self.joystick.get_axis(0)
                    y_left = self.joystick.get_axis(1)

                #Check backward command only
                while (trigger_left >= 0.5 and trigger_right <= 0.0):

                    #Check thumbstick control
                    #Account for backward heading orientation
                    if(abs(x_left) >= 0.6 or abs(y_left) >= 0.6):
                        #Get heading
                        final_degree = self.degreeHeading(x_left, y_left)

                        #Check left right signal
                        if (final_degree > 180 and final_degree <= 359):
                            self.rover_msg.data = "gospd -" + str(int(self.rover_speed * 0.1)) + " -" + str(int(self.rover_speed)) + "\r" #Only takes integers
                            self.rover_pub.publish(self.rover_msg)
                        else:
                            self.rover_msg.data = "gospd -" + str(int(self.rover_speed)) + " -" + str(int(self.rover_speed * 0.1)) + "\r" #Only takes integers
                            self.rover_pub.publish(self.rover_msg)

                        rospy.sleep(0.1)

                    #Just backward
                    else:
                        self.rover_msg.data = "gospd -" + str(int(self.rover_speed)) + " -" + str(int(self.rover_speed)) + "\r" #Only takes integers
                        self.rover_pub.publish(self.rover_msg)
                        rospy.sleep(0.1)

                    #Recheck values neglecting right trigger
                    pygame.event.get()
                    trigger_left = self.joystick.get_axis(2)
                    x_left = self.joystick.get_axis(0)
                    y_left = self.joystick.get_axis(1)

                #Make sure send stop signal after
                self.rover_msg.data = "go 0 0\r"
                self.rover_pub.publish(self.rover_msg)
                rospy.sleep(0.1)
                self.rover_pub.publish(self.rover_msg)
                rospy.sleep(0.1)

            #Checking only in place turning without moving forward or backward
            if ((abs(x_left) >= 0.6 or abs(y_left) >= 0.6) and (trigger_left <= 0.0 and trigger_right <= 0.0)):

                while ((abs(x_left) >= 0.6 or abs(y_left) >= 0.6)):
                    #Break if triggers are pressed to go into that mode
                    if(trigger_left >= 0.5 or trigger_right >= 0.5):
                        self.rover_msg.data = "go 0 0\r"
                        self.rover_pub.publish(self.rover_msg)
                        self.rover_msg.data = "go 0 0\r"
                        self.rover_pub.publish(self.rover_msg)
                        break

                    #Get heading
                    final_degree = self.degreeHeading(x_left, y_left)

                    #Check left signal
                    if (final_degree > 180 and final_degree <= 359):
                        self.rover_msg.data = "gospd -" + str(int(self.rover_speed)) + " " + str(int(self.rover_speed)) + "\r" #Only takes integers
                        self.rover_pub.publish(self.rover_msg)
                    else:
                        self.rover_msg.data = "gospd " + str(int(self.rover_speed)) + " -" + str(int(self.rover_speed)) + "\r" #Only takes integers
                        self.rover_pub.publish(self.rover_msg)

                    #Recheck values
                    pygame.event.get()
                    trigger_right = self.joystick.get_axis(5)
                    trigger_left = self.joystick.get_axis(2)
                    x_left = self.joystick.get_axis(0)
                    y_left = self.joystick.get_axis(1)
                    rospy.sleep(0.1)

                #Stop just in case keeps moving
                self.rover_msg.data = "go 0 0\r"
                self.rover_pub.publish(self.rover_msg)
                rospy.sleep(0.1)
                self.rover_pub.publish(self.rover_msg)
                rospy.sleep(0.1)

            #----------------------------------------------------------------------------------------------------
            #Pan/Tilt Commands
            #----------------------------------------------------------------------------------------------------
            #Check pad presses
            if (camera_pads[0] != 0 or camera_pads[1] != 0):

                #Check left or right
                while (camera_pads[0] != 0):

                    #Check direction
                    if (camera_pads[0] > 0): #Right
                        self.client.sendMessage("right")
                        time.sleep(0.1)

                    else: #Left
                        self.client.sendMessage("left")
                        time.sleep(0.1)

                    #Recheck values
                    pygame.event.get()
                    camera_pads = self.joystick.get_hat(0)

                #Check front or back
                while (camera_pads[1] != 0):

                    #Check direction
                    if (camera_pads[1] > 0): #Right
                        self.client.sendMessage("up")
                        time.sleep(0.1)

                    else: #Left
                        self.client.sendMessage("down")
                        time.sleep(0.1)

                    #Recheck values
                    pygame.event.get()
                    camera_pads = self.joystick.get_hat(0)

            #Make sure stop rover
            self.rover_msg.data = "go 0 0\r"
            self.rover_pub.publish(self.rover_msg)
            rospy.sleep(0.1)

            #Make sure stop arm
            self.arm_msg.z = 0.0
            self.arm_msg.y = 0.0
            self.arm_msg.x = 0.0
            self.arm_pub.publish(self.arm_msg)
            rospy.sleep(0.1)

            #Final ROS sleep rate
            self.rate.sleep()

        self.client.disconnect()
        pygame.quit()

#----------------------------------------------------------------------------------------------------
#Main Function start
#----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    joystick_controll = Joystick_Object()
    joystick_controll.runLoop()
