/*
  Arlo-Tune-Integral-Constants
  
  Set control system constants that give the extra push to get to the
  final position and control the allowable positional error.
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

  int ki = Arlo.readConfig("KI");             // Check KI
  Serial.print("KI = ");                      // Display KI
  Serial.println(ki);

  int dz = Arlo.readConfig("DZ");             // Check DZ
  Serial.print("DZ = ");                      // Display DZ
  Serial.println(dz);

  Arlo.writeConfig("KI", 65);                 // Change KI
  Arlo.writeConfig("DZ", 1);                  // Change DZ

  // Uncomment code below to store new settings to the DHB-10's EEPROM.
  // Tip: To uncomment, replace the two * characters with / characters. 
  
  /*
  Arlo.storeConfig("KI");                    // New KI to DHB-10 EEPROM
  Serial.println(Arlo.lastExchange);

  Arlo.storeConfig("DZ");                    // New DZ to DHB-10 EEPROM
  Serial.println(Arlo.lastExchange);
  */
}

void loop() {}                                // Nothing for main loop

