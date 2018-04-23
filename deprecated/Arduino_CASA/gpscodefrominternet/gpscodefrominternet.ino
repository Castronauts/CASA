#include <SoftwareSerial.h>
SoftwareSerial gps(2, 3);      // Rx=6, Tx=7
long timeout = 1000;           // Timeout if invalid data

#define GetLat    0x05         // Command for latitude
#define GetLong   0x06         // Command for longitude

void setup() {
  Serial.begin(9600);
  gps.begin(9600);
}

void loop() {
  delay(1000);
  getData(GetLat);
  Serial.print(",");
  getData(GetLong);
  Serial.println("");
}

void getData(byte gpsType) {
  if (sendGps(gpsType, 5) == 5) {
    byte degs = gps.read();
    byte mins = gps.read();
    long minsD = gps.read()<<8;
    minsD += gps.read();
    byte dir = gps.read();
    if(dir) { Serial.print("-"); }
    Serial.print(degs, DEC);
    Serial.print(".");
    long workVal = (mins * 1000 / 6) + (minsD / 60);
    Serial.print(workVal, DEC);
  }
}

byte sendGps (byte cmd, byte numBytes) {
  gps.print("!GPS");  
  gps.write(cmd);
  unsigned long oldMillis = millis();
  while(gps.available() < numBytes) {
    if (millis() - oldMillis > timeout)
      return (0);
  }
  return(gps.available());
}
