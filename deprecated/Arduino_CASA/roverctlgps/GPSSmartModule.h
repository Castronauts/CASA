
#ifndef GPS_SMART_MODULE_H__
#define GPS_SMART_MODULE_H__
/*
Files     	: GPSSmartModule.ino
Version 	: 1.0
Date     	: 2013/06/18
Description	: SimplyTronics ST-00059 GPS Smart Module test

History		:

    1. Date		: 2013/01/16
       Author		: John Zhong@SimplyTronics
       Modification	: Create
       	
    2. Date		: 2013/06/18
       Author		: John Zhong@SimplyTronics
       Modification	: Add the error check.	   

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

#define         CMD_INFO_BASIC             0x00    
#define         CMD_GET_INFO               0x00                                   
#define         CMD_GET_STATUS             0x01                       // Status          Status(8-bit) ## 0 #define Not Valid, 1 #define Valid 
#define         CMD_GET_SATELLITES         0x02                      // Satellites      Satellites(8-bit) ## 0 ~ 20
#define         CMD_GET_TIME               0x03                      // Time            Hour(8-bit), Minute(8-bit), Second(8-bit) ## 0 ~ 23, 0 ~ 59, 0 ~ 59  
#define         CMD_GET_DATE               0x04                      // Date            Year(8-bit), Month(8-bit), Day(8-bit) ## 00 ~ 99, 1 ~ 12, 1 ~ 31
#define         CMD_GET_LATITUDE           0x05                      // Latitude        Degree(8-bit), Minute(8-bit), Second(16-bit) ## 0~89, 0~59, 99.99
#define         CMD_GET_LONGITUDE          0x06                      // Longitude       Degree(8-bit), Minute(8-bit), Second(16-bit) ## 0~179, 0~59, 99.99
#define         CMD_GET_ALTITUDE           0x07                      // Altidute x 10   Alt(16-bit), 
#define         CMD_GET_SPEED_KMPH         0x08                      // Speed km/h x 10 Speed(16-bit)
#define         CMD_GET_SPEED_KNOT         0x09                      // Speed knot x 10 Speed(16-bit)
#define         CMD_GET_HEADING            0x0A                      // Course         
#define         CMD_SET_STATIC_NAVI_ON     0x0B			     // These two commands take 72 ms 
#define         CMD_SET_STATIC_NAVI_OFF    0x0C 		     //  please wait at least 72 ms before send next command
#define         CMD_RESET                  0x0D                                
                                               
                                                                  
#define         CMD_SET_POINT_BASIC        0x30                          // sub cmd  x = 1 ~ (TOTOAL_POINTS-1)   
                                                                           
#define         CMD_GET_DISTANCE_BASIC     0x40                          // sub cmd 0x00 ~ 0xxy, x = 1 ~ (TOTOAL_POINTS-1) , y = 1 ~ (TOTOAL_POINTS-1)                             

#define         CMD_DATALOGGER_BASIC       0x50        
#define         CMD_LOG_IDLE               0x00
#define         CMD_START_NEW_LOG          0x10
#define         CMD_PAUSE_LOG              0x20
#define         CMD_RESUME_LOG             0x30          
#define         CMD_GET_LOG_INFO           0x40                          // return the total records, this command will stop the data logger
#define         CMD_CLEAR_LOG_INFO         0x50                          // Only Clear the First page(4 bytes of the upper EERPOM)
#define         CMD_GET_LOG_DATA           0x60                          // return the total records, this command will stop the data logger
#define         CMD_GET_LOG_DATA_SLOW      0x70                          // return the total records, this command will stop the data logger
#define         CMD_LOG_FREQUENY_BASIC0    0xA0                          // 1s -- 1 hour
//#define         CMD_LOG_FREQUENY_BASIC1    0xB0                        

#define         CMD_GET_SPEED_AVER_BASIC     0x60                          // sub cmd 0x00 ~ 0xxy, x = 1 ~ (TOTOAL_POINTS-1) , y = 1 ~ (TOTOAL_POINTS-1)                             

#endif //GPS_SMART_MODULE_H__

