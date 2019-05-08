/*
  Arlo-Change-Pulse-Scale
  
  Change pulse scale from 1000...2000 us to 1300 to 1700 us so that 
  Robotics with the BOE Shield-Bot examples can run the Arlo at top speed. 
*/

#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>                   // Include SoftwareSerial library

// Arlo and serial objects required 
ArloRobot Arlo;                               // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

void setup()                                  // Setup function
{
  tone(4, 3000, 2000);                        // Piezospeaker beep
  Serial.begin(9600);                         // Start terminal serial port
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  Arlo.writeConfig("SCALE", 200);             // Change pulse to speed response
  //Arlo.writeConfig("SCALE", 1000);            // Want default back?  Un-comment
  Arlo.storeConfig("SCALE");                  // Save setting in EEPROM

  Arlo.writePulseMode();                      // Get ready to receive pulses
}

void loop() {}                                // Nothing for main loop

