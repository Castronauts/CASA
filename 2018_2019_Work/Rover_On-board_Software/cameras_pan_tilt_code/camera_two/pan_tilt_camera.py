from tcpcom import TCPServer
import sys
import time
import smbus

IP_PORT = 24000
sensitivity = 20
tilt = 900 # neutral position 900, max up 380, max down 1400 
bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
addr = 0x40           # I2C address of the PWM chip.
bus.write_byte_data(addr, 0, 0x20)     # enable the chip
bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write

bus.write_word_data(addr, 0x0A, 0)
bus.write_word_data(addr, 0x0C, tilt) 


def onStateChanged(state, msg):
    if state == "LISTENING":
        print "Server:-- Listening..."
    elif state == "CONNECTED":
        print "Server:-- Connected to", msg
    elif state == "MESSAGE":
        global tilt

        if msg == "up":
            if (tilt > 380):
                tilt -= sensitivity
                bus.write_word_data(addr, 0x0C, tilt)
        if msg == "down":
            if (tilt < 1400):
                tilt += sensitivity
                bus.write_word_data(addr, 0x0C, tilt)

if __name__ == '__main__':
    server = TCPServer(IP_PORT, stateChanged = onStateChanged)

