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
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Point
from dynamixel_msgs.msg import JointState

#----------------------------------------------------------------------------------------------------
#Global variables
#----------------------------------------------------------------------------------------------------
gripper_load = 0.0

#----------------------------------------------------------------------------------------------------
#Function Declarations
#----------------------------------------------------------------------------------------------------
def onStateChanged(state, msg):
    global isConnected
    if state == "CONNECTING":
       print "Client:-- Waiting for connection..."
    elif state == "CONNECTED":
       print "Client:-- Connection estabished."
    elif state == "DISCONNECTED":
       print "Client:-- Connection lost."
       isConnected = False

def updateGripperLoad(data):
    global gripper_load
    gripper_load = data.load

    print (gripper_load)

def degreeHeading(x, y):
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

#----------------------------------------------------------------------------------------------------
#Main Function start
#----------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    #----------------------------------------------------------------------------------------------------
    #Pan/Tilt Camera IP Socket
    #----------------------------------------------------------------------------------------------------
    IP_ADDRESS = "192.168.1.9"
    IP_PORT = 23000
    client = TCPClient(IP_ADDRESS, IP_PORT, stateChanged = onStateChanged)
    rc = client.connect()

    if(rc):
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
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    #----------------------------------------------------------------------------------------------------
    #ROS Initialization
    #----------------------------------------------------------------------------------------------------
    rospy.init_node("joystick_controller")

    #Initialize arm stuff
    arm_pub = rospy.Publisher('control_signal', Point, queue_size=10)
    arm_msg = Point()

    #Initialize gripper stuff
    gripper_pub = rospy.Publisher("/dual_gripper_controller/command", Float64, queue_size=10)
    rospy.Subscriber("/dual_gripper_controller/state", JointState, updateGripperLoad)
    gripper_msg = Float64()
    gripper_msg.data = 0.0
    gripper_pub.publish(gripper_msg)

    #Initialize rover stuff
    rover_pub = rospy.Publisher("arlo_wheels", String, queue_size=10)
    rover_msg = String()
    rover_speed = 30.0 #Out of 200 max
    rover_msg.data = "rst\r"
    rover_pub.publish(rover_msg)

    #Rate for loop
    rate = rospy.Rate(10) #10 Hertz

    #----------------------------------------------------------------------------------------------------
    #Main Loop
    #----------------------------------------------------------------------------------------------------
    while (not rospy.is_shutdown()):
        #Check user input
        pygame.event.get()

        #Get Joystick values
        x_left = joystick.get_axis(0)
        y_left = joystick.get_axis(1)
        trigger_left = joystick.get_axis(2)
        x_right = joystick.get_axis(3)
        y_right = joystick.get_axis(4)
        trigger_right = joystick.get_axis(5)

        #Get Hat Values
        camera_pads = joystick.get_hat(0) #[0]-left,right [1]-top,down

        #Get Button Values
        open_button = joystick.get_button(0) #A button
        close_button = joystick.get_button(1) #B button

        #Get bumper values
        arm_down = joystick.get_button(4) #Left bumper
        arm_up = joystick.get_button(5) #Right bumper
    
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
                    arm_msg.z = 1.0
                    arm_pub.publish(arm_msg)

                #Recheck values
                pygame.event.get()
                arm_down = joystick.get_button(4)
                arm_up = joystick.get_button(5)
    

                rospy.sleep(0.1)

            #Check only down
            while(arm_down != 0.0 and arm_up == 0.0):

                #Check thumbstick control
                if(abs(x_right) >= 0.6 or abs(y_right) >= 0.6):
                    print("Down with thumbstick")

                #Just down
                else:
                    arm_msg.z = -1.0
                    arm_pub.publish(arm_msg)

                #Recheck values
                pygame.event.get()
                arm_down = joystick.get_button(4)
                arm_up = joystick.get_button(5)
                
                rospy.sleep(0.1)

            rospy.sleep(0.1)

            #Turn off motion
            arm_msg.z = 0.0
            arm_msg.y = 0.0
            arm_msg.x = 0.0
            arm_pub.publish(arm_msg)

        #Check only joystick arm x and y
        if ((abs(x_right) >= 0.6 or abs(y_right) >= 0.6) and (arm_up == 0.0 and arm_down == 0.0)):

            while ((abs(x_right) >= 0.6 or abs(y_right) >= 0.6)):

                #Break if bumpers are pressed to go into that mode
                if(arm_up != 0.0 or arm_down != 0.0):
                    arm_msg.z = 0.0
                    arm_msg.y = 0.0
                    arm_msg.x = 0.0
                    arm_pub.publish(arm_msg)
                    rospy.sleep(0.1)
                    break

                #Get heading
                final_degree = degreeHeading(x_right, y_right)

                #Check forward signal
                if ((final_degree > 315 and final_degree <=359) or (final_degree >= 0 and final_degree <= 45)):
                    arm_msg.z = 0.0
                    arm_msg.y = 1.0
                    arm_msg.x = 0.0
                    arm_pub.publish(arm_msg)

                #Check right signal
                elif (final_degree > 45 and final_degree <= 135):
                    arm_msg.z = 0.0
                    arm_msg.y = 0.0
                    arm_msg.x = 1.0
                    arm_pub.publish(arm_msg)

                #Check backward signal
                elif (final_degree > 135 and final_degree <= 225):
                    arm_msg.z = 0.0
                    arm_msg.y = -1.0
                    arm_msg.x = 0.0
                    arm_pub.publish(arm_msg)

                #Check left signal
                elif (final_degree > 225 and final_degree <= 315):
                    arm_msg.z = 0.0
                    arm_msg.y = 0.0
                    arm_msg.x = -1.0
                    arm_pub.publish(arm_msg)
                
                #Recheck values
                pygame.event.get()
                arm_down = joystick.get_button(4)
                arm_up = joystick.get_button(5)
                x_right = joystick.get_axis(3)
                y_right = joystick.get_axis(4)

                rospy.sleep(0.1)

            #Shutdown values
            arm_msg.z = 0.0
            arm_msg.y = 0.0
            arm_msg.x = 0.0
            arm_pub.publish(arm_msg)
            rospy.sleep(0.1)


        #----------------------------------------------------------------------------------------------------
        #Gripper Commands
        #----------------------------------------------------------------------------------------------------
        #Check open close buttons
        if (open_button != 0.0 or close_button != 0.0):
            #Check open button only
            while (open_button != 0 and close_button == 0.0 and gripper_msg.data >= -0.4):
                #Send and sleep to repeat
                gripper_msg.data = gripper_msg.data - 0.01
                gripper_pub.publish(gripper_msg)
                rospy.sleep(0.02)

                #Recheck values
                pygame.event.get()
                open_button = joystick.get_button(0)

            #Check close button only
            while (close_button != 0 and open_button == 0.0 and gripper_msg.data <= 0.4 and gripper_load > -0.1):
                #Send and sleep to repeat
                gripper_msg.data = gripper_msg.data + 0.01
                gripper_pub.publish(gripper_msg)
                rospy.sleep(0.02)

                #Recheck values
                pygame.event.get()
                close_button = joystick.get_button(1)
            
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
                    final_degree = degreeHeading(x_left, y_left)

                    #Check left right signal
                    if (final_degree > 180 and final_degree <= 359):
                        rover_msg.data = "gospd " + str(int(rover_speed * 0.1)) + " " + str(int(rover_speed)) + "\r" #Only takes integers
                        rover_pub.publish(rover_msg)
                    else:
                        rover_msg.data = "gospd " + str(int(rover_speed)) + " " + str(int(rover_speed * 0.1)) + "\r" #Only takes integers
                        rover_pub.publish(rover_msg)

                    rospy.sleep(0.1)

                #Just forward
                else:
                    rover_msg.data = "gospd " + str(int(rover_speed)) + " " + str(int(rover_speed)) + "\r" #Only takes integers
                    rover_pub.publish(rover_msg)
                    rospy.sleep(0.1)
                    
                #Recheck values neglecting left trigger
                pygame.event.get()
                trigger_right = joystick.get_axis(5)
                x_left = joystick.get_axis(0)
                y_left = joystick.get_axis(1)

            #Check backward command only
            while (trigger_left >= 0.5 and trigger_right <= 0.0):

                #Check thumbstick control
                #Account for backward heading orientation
                if(abs(x_left) >= 0.6 or abs(y_left) >= 0.6):
                    #Get heading
                    final_degree = degreeHeading(x_left, y_left)

                    #Check left right signal
                    if (final_degree > 180 and final_degree <= 359):
                        rover_msg.data = "gospd -" + str(int(rover_speed * 0.1)) + " -" + str(int(rover_speed)) + "\r" #Only takes integers
                        rover_pub.publish(rover_msg)
                    else:
                        rover_msg.data = "gospd -" + str(int(rover_speed)) + " -" + str(int(rover_speed * 0.1)) + "\r" #Only takes integers
                        rover_pub.publish(rover_msg)

                    rospy.sleep(0.1)

                #Just backward
                else:
                    rover_msg.data = "gospd -" + str(int(rover_speed)) + " -" + str(int(rover_speed)) + "\r" #Only takes integers
                    rover_pub.publish(rover_msg)
                    rospy.sleep(0.1)

                #Recheck values neglecting right trigger
                pygame.event.get()
                trigger_left = joystick.get_axis(2)
                x_left = joystick.get_axis(0)
                y_left = joystick.get_axis(1)

            #Make sure send stop signal after
            rover_msg.data = "go 0 0\r"
            rover_pub.publish(rover_msg)
            rospy.sleep(0.1)
            rover_pub.publish(rover_msg)
            rospy.sleep(0.1)

        #Checking only in place turning without moving forward or backward
        if ((abs(x_left) >= 0.6 or abs(y_left) >= 0.6) and (trigger_left <= 0.0 and trigger_right <= 0.0)):

            while ((abs(x_left) >= 0.6 or abs(y_left) >= 0.6)):
                #Break if triggers are pressed to go into that mode
                if(trigger_left >= 0.5 or trigger_right >= 0.5):
                    rover_msg.data = "go 0 0\r"
                    rover_pub.publish(rover_msg)
                    rover_msg.data = "go 0 0\r"
                    rover_pub.publish(rover_msg)
                    break

                #Get heading
                final_degree = degreeHeading(x_left, y_left)

                #Check left signal
                if (final_degree > 180 and final_degree <= 359):
                    rover_msg.data = "gospd -" + str(int(rover_speed)) + " " + str(int(rover_speed)) + "\r" #Only takes integers
                    rover_pub.publish(rover_msg)
                else:
                    rover_msg.data = "gospd " + str(int(rover_speed)) + " -" + str(int(rover_speed)) + "\r" #Only takes integers
                    rover_pub.publish(rover_msg)
    
                rospy.sleep(0.1)

                #Recheck values
                pygame.event.get()
                trigger_right = joystick.get_axis(5)
                trigger_left = joystick.get_axis(2)
                x_left = joystick.get_axis(0)
                y_left = joystick.get_axis(1)

            #Stop just in case keeps moving
            rover_msg.data = "go 0 0\r"
            rover_pub.publish(rover_msg)
            rospy.sleep(0.1)
            rover_pub.publish(rover_msg)
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
                    client.sendMessage("right")
                    time.sleep(0.1)

                else: #Left
                    client.sendMessage("left")
                    time.sleep(0.1)

                #Recheck values
                pygame.event.get()
                camera_pads = joystick.get_hat(0)

            #Check front or back
            while (camera_pads[1] != 0):

                #Check direction
                if (camera_pads[1] > 0): #Right
                    client.sendMessage("up")
                    time.sleep(0.1)

                else: #Left
                    client.sendMessage("down")
                    time.sleep(0.1)

                #Recheck values
                pygame.event.get()
                camera_pads = joystick.get_hat(0)

        #Final ROS sleep rate
        rate.sleep()

        
    client.disconnect()
    pygame.quit()