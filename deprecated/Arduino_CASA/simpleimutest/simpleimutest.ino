#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>

//define addresses for accelerometer + mag, gyro (why the accel adn mag are the same address and the gryo isn't I don't know, would need to read datasheet more carefully)
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B


LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
void setup() {
  Serial.begin(9600); 
  uint16_t status = dof.begin(); // status returns whoami for gryo and accel. 
  if (status != 0x49D4) 
    Serial.println("SHIT IS FUCKED, ABORT");

}

void loop() {
  // put your main code here, to run repeatedly:

}
