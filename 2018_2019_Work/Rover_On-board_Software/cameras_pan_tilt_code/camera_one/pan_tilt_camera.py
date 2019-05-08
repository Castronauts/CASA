from tcpcom import TCPServer
import sys
import time
import smbus

IP_PORT = 23000
sensitivity = 20
pan = 970 # neutral position
tilt = 1120 # neutral position
bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
addr = 0x40           # I2C address of the PWM chip.
bus.write_byte_data(addr, 0, 0x20)     # enable the chip
bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write

bus.write_word_data(addr, 0x06, 0)     # chl 0 start time = 0us
bus.write_word_data(addr, 0x08, pan) # chl 0 end time = 1.5ms

bus.write_word_data(addr, 0x0A, 0)     # chl 1 start time = 0us
bus.write_word_data(addr, 0x0C, tilt) # chl 1 end time = 1.5ms


def onStateChanged(state, msg):
    if state == "LISTENING":
        print "Server:-- Listening..."
    elif state == "CONNECTED":
        print "Server:-- Connected to", msg
    elif state == "MESSAGE":
        global pan
        global tilt

        if msg == "up":
            if (pan < 1800):
                pan += sensitivity
                bus.write_word_data(addr, 0x08, pan)
        if msg == "down":
            if (pan > 410):
                pan -= sensitivity
                bus.write_word_data(addr, 0x08, pan)
        if msg == "right":
            if (tilt > 400):
                tilt -= sensitivity
                bus.write_word_data(addr, 0x0C, tilt)
        if msg == "left":
            if (tilt < 1900):
                tilt += sensitivity
                bus.write_word_data(addr, 0x0C, tilt)

if __name__ == '__main__':
    server = TCPServer(IP_PORT, stateChanged = onStateChanged)

