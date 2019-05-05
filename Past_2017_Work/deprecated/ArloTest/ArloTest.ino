#include <ArloRobot.h>
#include <SoftwareSerial.h>

// Arlo and serial objects required
ArloRobot Arlo;                               // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

int countsLeft, countsRight;                  // Encoder counting variables
void setup()                                  // Setup function
{
  //tone(4, 3000, 2000);                        // Piezospeaker beep
  delay(3000);                                // Wait three seconds
  Serial.begin(9600);                         // Start terminal serial port
  Serial.println("Program running...");       // Display starting message
 
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object

  Arlo.clearCounts();                         // Clear encoder counts

  Arlo.writeCounts(544, 544);                 // Go forward 144 counts
  delay(5000);                                // Wait three seconds
  displayDistances();                         // Display encoder counts

  Arlo.writeCounts(-72, 72);                  // Turn left 72 counts
  delay(2000);                                // Wait two seconds
  displayDistances();                         // Display encoder counts

  Arlo.writeCounts(72, -72);                  // Turn right 72 counts
  delay(2000);                                // Wait 2 seconds
  displayDistances();                         // Display encoder counts

  Arlo.writeCounts(-544, -544);               // Back up 144 counts
  delay(3000);                                // Wait three seconds
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
