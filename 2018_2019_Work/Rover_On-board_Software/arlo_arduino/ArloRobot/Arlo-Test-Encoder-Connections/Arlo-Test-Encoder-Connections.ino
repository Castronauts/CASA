/*
  Arlo-Test-Encoder-Connections
  
  This sketch tests to make sure the Arlo's wheel encoder connections
  are correct.  The Arlo will not be ready for the next step until you 
  have verified that the number of encoder transitions (ticks) for both
  wheels are positive when the wheels roll forward.
  
  If you have not already completed Test Arlo Motor Connections.c, 
  complete it first, then continue from here.
  
  Use the Arduino IDE’s Upload button to run this sketch.  If the
  Terminal displays the "Encoder connections are correct!..."
  message, your Arlo is ready for the next step, which is running 
  navigation sketches.  
  
  If the Terminal instead displays one or more "ERROR..."
  messages, those encoder encoder connections will need to be 
  corrected.  For example, if the messages says, "ERROR: Motor 1 
  encoder connections are reversed!", you will need to unplug and
  swap the two 3-wire encoder cables next to the Motor 1 terminal
  on the DHB-10, swap them, and plug them back in.
  
  Make sure to test between each adjustment.  Your arlo will not be
  ready for the next step until you get the success message from
  this test. 
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
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  Serial.println("Testing...");               // Display testing message

  Arlo.clearCounts();                         // Clear encoder counts

  Arlo.writeMotorPower(32, 32);               // Go forward very slowly
  delay(4000);                                //   for three seconds
  Arlo.writeMotorPower(0, 0);                 //   then stop
  
  countsLeft = Arlo.readCountsLeft();         // Get left & right encoder counts
  countsRight = Arlo.readCountsRight();

  Serial.print("countsLeft = ");              // Display encoder measurements 
  Serial.print(countsLeft, DEC);
  Serial.print(", countsRight = ");
  Serial.println(countsRight, DEC);
  Serial.println();

  // Both distances positive?    
  if((countsLeft > 175) && (countsLeft < 325) 
  && (countsRight > 175) && (countsRight < 325))
  {
    // Success message
    Serial.println("Encoder connections are correct!");
    Serial.println("Your Arlo is ready for the next step.");
    Serial.println();
  } 
  else
  { 
    // Left encoders cables correct?  
    if(countsLeft > 175 && countsLeft < 325)     
    {
      // Correct encoder message
      Serial.println("Motor 1 encoder cables are connected");
      Serial.println(" correctly.");
      Serial.println();
    }    
    // Left encoders cables swapped?
    else if(countsLeft > -325 && countsLeft < -125)   
    {
      // Swapped encoder message
      Serial.println("ERROR: Motor 1 encoder connections");
      Serial.println(" are reversed!");
      Serial.println();
    }    
    else                                      // Other problem
    {
      // Other encoder error message
      Serial.println("ERROR: Motor 1 encoder values out of ");
      Serial.println("range. Recheck encoder connections ");
      Serial.println("and assemblies.");
      Serial.println();      
    }   
    // Right encoders cables correct?      
    if(countsRight > 175 && countsRight < 325)
    {
      // Correct encoder message
      Serial.println("Motor 2 encoder cables are ");
      Serial.println("connected correctly.");
      Serial.println();
    } 
    // Right encoders cables swapped?   
    else if(countsRight > -325 && countsRight < -125)
    {
      // Swapped encoder message
      Serial.println("ERROR: Motor 2 encoder connections ");
      Serial.println("are reversed!");
      Serial.println();
    } 
    else                                      // Other problem
    {
      // Other encoder error message
      Serial.println("ERROR: Motor 2 encoder values "); 
      Serial.println("out of range. Recheck encoder ");
      Serial.println("connections and assemblies.");
      Serial.println();
    }      
  }    

  Arlo.writePulseMode();                      // Get ready for pulse control

  Serial.print("Test done.\n\n");             // Display status
}

void loop() {}                                // Nothing for main loop

