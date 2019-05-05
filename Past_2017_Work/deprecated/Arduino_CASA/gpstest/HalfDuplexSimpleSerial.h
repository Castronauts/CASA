#ifndef HALF_DUPLEX_SIMPLE_SERIAL_H__
#define HALF_DUPLEX_SIMPLE_SERIAL_H__
/*
Files     	: HalfDuplexSimpleSerial.c
Version 	: 1.0
Date     	: 2013/01/16
Description	: This is a Half Duplex Serial Driver, The Driver only tested on SimplyTronics GPS Smart Module @ 9600 bps and UNO
                  The driver needs a pull-up resisor, please add a 4.7k ~ 100 k resistor to the SIO
                  (The GPS Smart Module already has Pull-up resistor)
                  The driver is basic on the original SoftwareSerial object from the library. 
                  Re-write with C. Did some changes to the low level, inline assembly language, pins configure etc...
                  Because the serial drive need accurate timing, so no big change to the original code to avoid creating the look up table.

                 Note: The driver will block the program while waiting for the data. This global interrupt has been disable, and will be resume
                       before exit the function.
                 
History		:

    1. Date		: 2013/01/16
       Author		: John Zhong@SimplyTronics
       Modification	: Create
   
    2. Date		: 2013/06/18
       Author		: John Zhong@SimplyTronics
       Modification	: Fixed the timeout bug, add the Error check.
   
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/


#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Arduino.h"

#define ERROR_RX_TIMEOUT ((uint16_t)0xFFFF) 

#ifdef __cplusplus
extern "C" {
#endif

extern void HDSS_start(uint8_t rxtxPin, uint8_t inverse_logic, long speed);
extern void HDSS_write(uint8_t byte);
extern uint8_t HDSS_read(void);
/* receive a byte with a simple implement of timeout */
extern uint16_t HDSS_read_time(uint32_t timeout);
extern void HDSS_stop(void);

#ifdef __cplusplus
}
#endif
#endif //HALF_DUPLEX_SIMPLE_SERIAL_H__
