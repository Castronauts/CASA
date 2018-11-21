#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
#import rospy

from tcpcom import TCPClient

IP_ADDRESS = "192.168.1.9"
IP_PORT = 23000

def onStateChanged(state, msg):
    global isConnected
    if state == "CONNECTING":
       print "Client:-- Waiting for connection..."
    elif state == "CONNECTED":
       print "Client:-- Connection estabished."
    elif state == "DISCONNECTED":
       print "Client:-- Connection lost."
       isConnected = False
    elif state == "MESSAGE":
       print "Client:-- Received data:", msg

if __name__ == '__main__':
    client = TCPClient(IP_ADDRESS, IP_PORT, stateChanged = onStateChanged)
    rc = client.connect()
    if rc:
        isConnected = True
        while isConnected:
            print "Client:-- Sending command: go..."
            client.sendMessage("go")
            time.sleep(2)
        client.disconnect()    
    else:
        client.disconnect()
        print "Client:-- Connection failed"