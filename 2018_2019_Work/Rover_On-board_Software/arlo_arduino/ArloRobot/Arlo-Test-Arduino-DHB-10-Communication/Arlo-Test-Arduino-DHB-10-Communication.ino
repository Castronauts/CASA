/*
  Arlo-Test-Arduino-DHB-10-Communication
  
  Insert web address for instructions here.
  
  You'll re ready for the next step when the Arduino Terminal displays:

  fwver = 10 
  hwver = 1
*/

#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>                   // Include SoftwareSerial library

ArloRobot Arlo;                               // Declare Arlo object
SoftwareSerial ArloSerial(12, 13);            // Declare SoftwareSerial object
                                              // DHB-10 -> I/O 12, DHB-10 <- I/O 13
void setup()                                  // Setup function
{
  tone(4, 3000, 2000);                        // Piezospeaker beep
  Serial.begin(9600);                         // Start terminal serial port

  ArloSerial.begin(19200);                    // Start DHB-10 serial communication
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  int fwver = Arlo.readFirmwareVer();         // Check DHB-10 firmware
  Serial.print("fwver = ");                   // Display firmware version
  Serial.println(fwver, DEC);

  int hwver = Arlo.readHardwareVer();         // Check DHB-10 hardware
  Serial.print("hwver = ");                   // Display hardware version
  Serial.println(hwver, DEC);
}

void loop() {}                                // Nothing for main loop
