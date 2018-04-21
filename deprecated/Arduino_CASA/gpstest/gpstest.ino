#include "HalfDuplexSimpleSerial.h"
#include "GPSSmartModule.h"
#include "ParallaxSerialTerminal.h"
#include "gps_type_define.h"
#include "lunargps.c"

#define         GPS_SIO                    (12) //Which pin is GPS plugged in to? 
#define         GPS_BAUDRATE               (9600)
#define         SERIAL_MONITOR_BR           (9600)

static uint8_t set_point_flag = 0; // Set Point Flag, indicates the original position(after power up,the GPS get the current position)
static uint8_t signal_valid_flag = 0; // signal valid

// types are re-defined for the following variants, please find the infomation in file "gps_type_define.h".
utc_t utc;// UTC time, union, hour, minute, second
date_t date; // date, union, day, month, year
version_t ver;// version info, union, hardware ver, firmware ver
satellite_t satellites; // satellites, 8-bit val
latitude_t latitude; // latitude, union, degree, minute, second(the module will return degree(8-bit), minute(8-bit), mill-minute(16-bit))
longitude_t longitude;// longitude, union, degree, minute, second(the module will return degree(8-bit), minute(8-bit), mill-minute(16-bit))
altitude_t altitude;// altitude, union, 16-bit value, -3276.8~3276.7 meters
speed_knot_t speed_knot;// speed, union, 16-bit value, 0~6553.5 knots
heading_t heading; // direction of travel to North, union, 16-bit value, 0~359.9 degree
distance_t distance; // distance between two set points, 32-bit value, 0~4294967294 meters(4294967295 == -1, error occured)
speed_average_t speed_average; // average speed between two set points,, 32-bit value, 0~4294967294 m/s(4294967295 == -1, error occured)



void setup() {
  Serial.begin(9600);
  HDSS_start(18, 0, GPS_BAUDRATE); //initialize the half duplex software serial driver

}

void loop() {

  uint8_t latbuffer[5]; //max response size is 5 bytes
  uint8_t longbuffer[5];
  uint8_t altbuffer[2];
  uint8_t timebuffer[3];
  uint16_t timeout;
  uint16_t altitude;
  int i;

  //this combination of actions will be put in one function
  /*timeout = gps_get_lat(responsebuffer);
  if(timeout == ERROR_RX_TIMEOUT)
    Serial.println("response timed out!");*/
  gps_get_lat(latbuffer);
  gps_get_long(longbuffer);
  gps_get_alt(altbuffer);
  gps_get_time(timebuffer);
  // Serial.print("Latitude: ");
  for (i = 0; i < 5; i++)
  {
    Serial.print(latbuffer[i]);
    Serial.print(" ");
  }
  Serial.print(" ");
  // Serial.print("Longitude: ");
  for (i = 0; i < 5; i++)
  {
    Serial.print(longbuffer[i]);
    Serial.print(" ");
  }
  Serial.print(" ");
  //Serial.print("Altitude: ");
  altitude = altbuffer[1] | (altbuffer[0] << 8);
  Serial.print(altitude);

  Serial.print(" ");
  // Serial.print("Time: ");
  for (i = 0; i < 3; i++)
  {
    Serial.print(":");
    Serial.print(timebuffer[i]);
  }
  Serial.print(" ");
  Serial.println();

  delay(1000);



}
