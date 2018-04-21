#ifndef PARALLAX_SERIAL_TERMINAL_H__
#define PARALLAX_SERIAL_TERMINAL_H__
/*
Files     	: ParallaxSerialTerminal.h
Version 	: 1.0
Date     	: 2013/01/16
Description	: This is the head file for Parallax Serial Terminal control commands
History		:

    1. Date		: 2013/01/16
       Author		: John Zhong@SimplyTronics
       Modification	: Create
	   

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


#define         PST_CONTROL_CLEAR           (0)
#define         PST_CONTROL_HOME            (1)
#define         PST_CONTROL_POSITION_X_Y    (2)
#define         PST_CONTROL_CURSOR_LEFT     (3)
#define         PST_CONTROL_CURSOR_RIGHT    (4)
#define         PST_CONTROL_CURSOR_UP       (5)
#define         PST_CONTROL_CURSOR_DOWN     (6)
#define         PST_CONTROL_BEEP            (7) 
#define         PST_CONTROL_BACKSPACE       (8)
#define         PST_CONTROL_TAB             (9)
#define         PST_CONTROL_LINE_FEED       (10)
#define         PST_CONTROL_CLEAR_END_OF_LINE (11)
#define         PST_CONTROL_CLEAR_LINE_BELOW  (12)
#define         PST_CONTROL_NEW_LINE        (13)
#define         PST_CONTROL_POSITION_X      (14)
#define         PST_CONTROL_POSITION_Y      (15)

#endif //PARALLAX_SERIAL_TERMINAL_H__
