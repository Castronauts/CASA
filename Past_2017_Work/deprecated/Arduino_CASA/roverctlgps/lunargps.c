#include "GPSSmartModule.h"
#include <inttypes.h>
#include "HalfDuplexSimpleSerial.h"

void gps_send_cmd(uint8_t cmd_b, uint8_t cmd_s) //basic function to send a command to module
{  
  HDSS_write(0xA5); //half duplex software serial write
  HDSS_write(cmd_b);
  HDSS_write(cmd_s);
  HDSS_write(0x5A);
}

uint16_t gps_receive_response(uint8_t num, uint8_t* buff) //NOTE: Buff MUST be at least size num or will segfault. Returns Error code, or 0 on success
{
  uint32_t rxdata; // 4 bytes on error, 1 byte (LSB) otherwise. Will include error checking so we can debug on rover.
  int i; 
  for(i = 0; i < num; i++)
  {
      rxdata = HDSS_read_time(8); //this is the timeout they include in the example code so I assume it's a reasonable value.
      if (rxdata == ERROR_RX_TIMEOUT)
      {
        return ERROR_RX_TIMEOUT; 
      }

      buff[i] = rxdata; 
  }
  return 0;
}

uint16_t gps_get_time(uint8_t* buff) //store current time in buffer in format HOURS: MIN : SEC
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_TIME);
  success = gps_receive_response(3, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

uint16_t gps_set_point(uint8_t loc)
{
  uint8_t response[1]; 
  gps_send_cmd(CMD_SET_POINT_BASIC,loc);
  gps_receive_response(1, response);
  return (uint16_t)(*response);
}

uint16_t gps_get_distance(uint8_t loc0, uint8_t loc1, uint8_t* buff) //gets the distance between two points. We'll offload this to the GPS coprocessor to save arduino resources
{
  int success; 
  gps_send_cmd(CMD_GET_DISTANCE_BASIC, (loc1<<4) | loc0); //cmd is in format 0xloc1loc0
  success = gps_receive_response(4,buff);
  return success; 
}

uint16_t gps_get_lat(uint8_t* buff)
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_LATITUDE);
  success = gps_receive_response(5, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

uint16_t gps_get_long(uint8_t* buff)
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_LONGITUDE);
  success = gps_receive_response(5, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

uint16_t gps_get_alt(uint8_t* buff)
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_ALTITUDE);
  success = gps_receive_response(2, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

uint16_t gps_get_speed(uint8_t* buff) //returns speed in knots
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_SPEED_KNOT);
  success = gps_receive_response(2, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

uint16_t gps_get_head(uint8_t* buff)
{
  int success; 
  gps_send_cmd(CMD_INFO_BASIC, CMD_GET_HEADING);
  success = gps_receive_response(2, buff); //Time response is 3 long, hours, min, sec in UTC
  return success; 
}

