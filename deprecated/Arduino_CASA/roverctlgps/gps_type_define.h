
#ifndef GPS_TYPE_DEFINE_H__
#define GPS_TYPE_DEFINE_H_
/*
Files     	: GPS_type_define.h
Version 	: 1.0
Date     	: 2013/06/16
Description	: A simple implement of low speed simple serial drive for propeller.
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

/*
type re-define
re-define the type of data to make it easy to access with program, and easy to be read.

In side the "union" type, defined two structs with same size, and they share the same memory(RAM) in the MCU.
(In other word, they only have differen names, you can accesse them with different names, but will point to the same memory). 

*/
typedef union
{
    struct
    {
        uint8_t buff[3];
    };
    struct
    {
        uint8_t hour;
        uint8_t minute;
        uint8_t second;

    };
} utc_t;



typedef union
{
    struct
    {
        uint8_t buff[2];
    } ;
    struct
    {
        uint8_t hardware_ver;
        uint8_t firmware_ver;
    } ;
} version_t;

typedef union
{
    struct
    {
        uint8_t buff[3];
    } ;
    struct
    {
        uint8_t day;
        uint8_t month;
        uint8_t year;
    };
} date_t;


typedef union
{
    struct
    {
        uint8_t buff[5];
    } ;
    struct
    {
        uint8_t degree;
        uint8_t minute;
        uint16_t second;
        uint8_t indicator;
    };
} latitude_t;


typedef union
{
    struct
    {
        uint8_t buff[5];
    } ;
    struct
    {
        uint8_t degree;
        uint8_t minute;
        uint16_t second;
        uint8_t indicator;
    };
} longitude_t;

typedef union
{
    struct
    {
        uint8_t buff[2];
    } ;
    int16_t altitude;

} altitude_t;

typedef union
{
    struct
    {
        uint8_t buff[2];
    } ;
    int16_t speed;

} speed_knot_t;

typedef union
{
    struct
    {
        uint8_t buff[2];
    } ;
    uint16_t degree;

} heading_t;

typedef union
{
    struct
    {
        uint8_t buff[4];
    } ;
    uint32_t distance;

} distance_t;


typedef union
{
    struct
    {
        uint8_t buff[4];
    } ;
    uint32_t speed;

} speed_average_t;

typedef uint16_t satellite_t;

#endif // GPS_TYPE_DEIFNE_H

