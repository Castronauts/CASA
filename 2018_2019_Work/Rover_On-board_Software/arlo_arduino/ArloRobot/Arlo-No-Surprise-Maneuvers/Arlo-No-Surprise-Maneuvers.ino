/*
  Arlo-No-Surprise-Maneuvers.ino
  
  Run this before ever turning on power to the Arlo's motors to prevent any
  unexpected motions.
*/  

void setup()                                    // Setup function
{
  tone(4, 3000, 2000);                          // Piezospeaker beep
  Serial.begin(9600);                           // Start terminal serial port
  Serial.print("Your Arlo will stay still.");   // Message
}
void loop() {}                                  // Nothing for main loop

