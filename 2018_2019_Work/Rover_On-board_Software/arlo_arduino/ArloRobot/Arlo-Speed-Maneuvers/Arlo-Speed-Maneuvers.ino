/*
  Arlo-Speed-Maneuvers
  
  Examples that use ArloRobot library to make the arlo travel in certain
  speeds for certain amounts of time.
*/

#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>                   // Include SoftwareSerial library

// Arlo and serial objects required 
ArloRobot Arlo;                               // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

int countsLeft, countsRight;                  // Encoder counting variables
void setup()                                  // Setup function
{
  tone(4, 3000, 2000);                        // Piezospeaker beep
  Serial.begin(9600);                         // Start terminal serial port
  Serial.println("Sketch running...");        // Display starting message
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  Arlo.clearCounts();                         // Clear encoder counts

  Arlo.writeSpeeds(144, 144);                 // Go forward 144 counts/sec
  delay(3000);                                // for three seconds
  Arlo.writeSpeeds(0, 0);                     // Stop
  delay(1000);                                // for one second
  displayDistances();                         // Display encoder counts

  Arlo.writeSpeeds(-72, 72);                  // Rotate seft 72 counts/sec
  delay(2000);                                // for two seconds
  Arlo.writeSpeeds(0, 0);                     // Stop
  delay(1000);                                // ...for one second
  displayDistances();                         // Display encoder counts

  Arlo.writeSpeeds(72, -72);                  // Rotate right counts/sec
  delay(2000);                                // for two seconds
  Arlo.writeSpeeds(0, 0);                     // Stop
  delay(1000);                                // for one second
  displayDistances();                         // Display encoder counts

  Arlo.writeSpeeds(-144, -144);               // Go backward 144 counts/sec
  delay(3000);                                // for three seconds
  Arlo.writeSpeeds(0, 0);                     // Stop
  delay(1000);                                // for one second
  displayDistances();                         // Display encoder counts
}

void loop() {}                                // Nothing for main loop

void displayDistances()
{
  countsLeft = Arlo.readCountsLeft();         // Get left & right encoder counts
  countsRight = Arlo.readCountsRight();

  Serial.print("countsLeft = ");              // Display encoder measurements 
  Serial.print(countsLeft, DEC);
  Serial.print(", countsRight = ");
  Serial.println(countsRight, DEC);
}

