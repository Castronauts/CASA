/*
  Arlo-Terminal-Communication
  
  IMPORTANT: Set Line Ending to Carriage return in the terminal.
*/

#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>                   // Include SoftwareSerial library

// Arlo and serial objects required 
ArloRobot Arlo;                               // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

char str[64];

void setup()                                  // Setup function
{
  tone(4, 3000, 2000);                        // Piezospeaker beep
  Serial.begin(9600);                         // Start terminal serial port
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object
  
  Serial.println("Arlo Terminal");            // Display heading
}

void loop()                                   // Main loop
{
  memset(str, 0, 64);                         // Clear the buffer
  Serial.print("> ");                         // Display prompt
  while(Serial.available() == 0);             // Wait for terminal input
  Serial.readBytesUntil('\r', str, 64);       // Read until carriage return
  Serial.println(str);                        // Display what was typed
  ArloSerial.print(str);                      // Send to Arlo's DHB-10
  ArloSerial.write('\r');                     // Append with a carriage return
  memset(str, 0, 64);                         // Clear the buffer again
  ArloSerial.readBytesUntil('\r', str, 64);   // Get Arlo's reply
  Serial.println(str);                        // Display Arlo's reply
}
