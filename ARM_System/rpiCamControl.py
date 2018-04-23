from getch import getch
import smbus, time

sensitivity = 5
pan = 1250 # neutral position
tilt = 1250 # neutral position

bus = smbus.SMBus(1)  # the chip is on bus 1 of the available I2C buses
addr = 0x40           # I2C address of the PWM chip.
bus.write_byte_data(addr, 0, 0x20)     # enable the chip
bus.write_byte_data(addr, 0xfe, 0x1e)  # configure the chip for multi-byte write

bus.write_word_data(addr, 0x06, 0)     # chl 0 start time = 0us
bus.write_word_data(addr, 0x08, pan) # chl 0 end time = 1.5ms

bus.write_word_data(addr, 0x0A, 0)     # chl 1 start time = 0us
bus.write_word_data(addr, 0x0C, tilt) # chl 1 end time = 1.5ms

## The 1.5ms end time is equal to 1.2us per count. This represents the neutral
## position of the servo, midway between both extremes. Each degree of
## deviation from neutral requires that number (1250) to be changed by 4.6.
## Thus, the 90 degree offset from neutral requires that 414 counts (90*4.6)
## be added or subtracted from 1250.

# look for arrow inputs to use for rpi servo control
while True:
    key1 = getch()
    if (ord(key1) != 27):
        continue
    #print('First char: {}'.format(ord(key)))
    key2 = getch()
    if (ord(key2) != 91):
        continue
    #print('Second char: {}'.format(ord(key)))
    key3 = getch()
    #print('Third char: {}'.format(ord(key)))

    if (ord(key3) == 65):
        #print('Up')
        pan += sensitivity
        bus.write_word_data(addr, 0x08, pan)
        continue
    if (ord(key3) == 66):
        #print('Down')
        pan -= sensitivity
        bus.write_word_data(addr, 0x08, pan)
        continue
    if (ord(key3) == 67):
        #print('Right')
        tilt += sensitivity
        bus.write_word_data(addr, 0x0C, tilt)
        continue
    if (ord(key3) == 68):
        #print('Left')
        tilt -= sensitivity
        bus.write_word_data(addr, 0x0C, tilt)
