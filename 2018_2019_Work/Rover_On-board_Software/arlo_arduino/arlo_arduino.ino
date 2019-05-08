#include <ArloRobot.h>                        // Include Arlo library
#include <SoftwareSerial.h>                   // Include SoftwareSerial library

// Arlo and serial objects required 
ArloRobot Arlo;                               // Arlo object
SoftwareSerial ArloSerial(12, 13);            // Serial in I/O 12, out I/O 13

char str[64];
int adc = 0;
double old_voltage = 0;
double new_voltage = 0;
double adc_scale = double(5.0 / 1023.0); //1024 10-bit but some error compared to voltmeter so set higher

void setup()                                  // Setup function
{
  Serial.begin(9600);                         // Start terminal serial port for xBee com
  
  ArloSerial.begin(19200);                    // Start DHB-10 serial com
  Arlo.begin(ArloSerial);                     // Pass to Arlo object
  
  Serial.println("Arlo Terminal");            // Display heading
}

void loop()                                   // Main loop
{
  memset(str, 0, 64);                         // Clear the buffer

  if (Serial.available() == 0) //Continue outputting voltage values
  {
    adc = analogRead(0);

    old_voltage = adc * adc_scale; //Original 5V ADC conversion value 10-bit

    new_voltage = old_voltage * (12.0 / 5.0); //Now need to convert this 5V volt reading into 12V basis for actual battery level. 
    
    Serial.println(new_voltage, 3);
  }
  else //Got input from user send commands accordingly
  {
    Serial.readBytesUntil('\r', str, 64);       // Read until carriage return
    Serial.println(str);                        // Display what was typed
    ArloSerial.print(str);                      // Send to Arlo's DHB-10
    ArloSerial.write('\r');                     // Append with a carriage return
    memset(str, 0, 64);                         // Clear the buffer again
    ArloSerial.readBytesUntil('\r', str, 64);   // Get Arlo's reply
  }
}
