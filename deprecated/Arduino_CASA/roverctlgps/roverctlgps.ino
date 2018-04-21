#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include "HalfDuplexSimpleSerial.h"
#include "GPSSmartModule.h"
#include "ParallaxSerialTerminal.h"
#include "gps_type_define.h"
#include "lunargps.c"

#define BRAKESPEED 2 //what fraction of the moving speed to brake at 
#define LSM9DS0_XM 0x1D
#define LSM9DS0_G  0x6B
#define T_DELAY_ms 200
#define T_DELAY_s (T_DELAY_ms/1000)
#define HP_COEF 0.6
#define LP_COEF (1-HP_COEF)
#define xOFF 5
#define yOFF -1
#define LBOUND 544
#define UBOUND 575
#define         GPS_SIO                    (18) //Which pin is GPS plugged in to? CAN'T Be 12.  
#define         GPS_BAUDRATE               (9600)
#define         SERIAL_MONITOR_BR           (9600)

const int pinDir = 5;
const int pinPWM = 10;
const int pinServo = 7;
const int pinCamxPWM = 3; //this is A
const int pinCamxDir = 12;
const int pinCamyPWM = 11; //this is B
const int pinCamyDir = 13;
int lastxspeed = 0;
int lastyspeed = 0;
int lastxdir = 0;
int lastydir = 0;
float x_angle; // Angle of tilt parallel to long edge
float y_angle; // Angle of tilt parallel to short edge
float pitch;
float roll;
uint8_t latbuffer[5]; //max response size is 5 bytes
uint8_t longbuffer[5]; 
uint8_t altbuffer[2];
uint8_t timebuffer[3];

Servo steer;
//LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

void readGPS(void)
{
  gps_get_lat(latbuffer);
  gps_get_long(longbuffer);
  gps_get_alt(altbuffer);
  gps_get_time(timebuffer);
  return; 
}

void readGyro(void) //reads the Gyro and updates x_angle and y_angle, as labeled on LSM9DS0. Time to read is negligible
{
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  calcOrientation(accel.acceleration.x,accel.acceleration.y,accel.acceleration.z);
  y_angle = (HP_COEF) * (y_angle +
                         (gyro.gyro.y) * T_DELAY_s) + ((LP_COEF) * pitch);
  x_angle = (HP_COEF) * (x_angle +
                         (gyro.gyro.x) * T_DELAY_s) + ((LP_COEF) * roll);
  return;
}

void calcOrientation(float x, float y, float z)
{
  pitch = atan2(x, sqrt((y * y) + (z * z)));
  roll = atan2(y, sqrt((x * x) + (z * z)));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  return;
}

void centerCamera(void)
{
  analogWrite(pinPWM, 0); //stop robot, because we won't have control of it while camera is centering
  digitalWrite(pinDir, HIGH);

  int counter = millis(); 
  int counter1; 

  int potvalue = analogRead(A5);

  while ((potvalue < LBOUND) || (potvalue > UBOUND)) {
    if (potvalue < 559) {
      do {
        analogWrite(pinCamyPWM, 64);
        digitalWrite(pinCamyDir, LOW);
        delay(10); 
        potvalue = analogRead(A5);
      }
      while (potvalue < 559);
      analogWrite(pinCamyPWM,0); 
    }
    if (potvalue > 559)
    {
      do {
        analogWrite(pinCamyPWM, 64);
        digitalWrite(pinCamyDir, HIGH);
        delay(10); 
        potvalue = analogRead(A5);
      }
      while (potvalue > 559); 
      analogWrite(pinCamyPWM, 0); 
    }
   delay(15);
   potvalue = analogRead(A5);   
   counter1 = millis(); 
   if((counter1 - counter) > 1000)
    break; 
  }

  return; 
}



void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000); //don't wait longer than the joystick sends commands, should prevent overflow
  HDSS_start(GPS_SIO, 0, GPS_BAUDRATE);


  pinMode(pinDir, OUTPUT);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinServo, OUTPUT);
  steer.attach(pinServo);
  lsm.begin(); 
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
 

  centerCamera();
  delay(2000);
}

void loop() {
  int xpos;
  int ypos;
  //SERIAL STUFF HERE
  //first we suck up the whole line
  String input;
  String x;
  String y;
  String camx;
  String camy;
  int camxpos;
  int camypos;
  int firstspace;
  int secondspace;
  int thirdspace;
  uint16_t timeout; 
  uint16_t altitude; 
  int i; 



WAIT_SERIAL:
  input = Serial.readStringUntil('\n'); //this should in theory read until \n or timeout. HOPEFULLY timeout is long enough to not eat data
  readGyro();
  readGPS();
  if (input == "") { //no command received, exited on timeout
    analogWrite(pinPWM, 0); //stop running away
    digitalWrite(pinDir, HIGH);
    goto WAIT_SERIAL; //fuck you, code elitists
  }
  Serial.print(x_angle); //send IMU data back to operator
  Serial.print(" ");
  Serial.print(y_angle);
  Serial.print(" "); 
  for(i = 0; i < 5; i++)
  {
    Serial.print(latbuffer[i]); 
    Serial.print(" "); 
  }
  for(i = 0; i < 5; i++)
  {
    Serial.print(longbuffer[i]);
    Serial.print(" ");  
  }
  altitude = altbuffer[1] | (altbuffer[0]<<8); 
  Serial.print(altitude); 
  Serial.print(" "); 
  for(i = 0; i < 3; i++)
  {
    Serial.print(timebuffer[i]); 
  }
  Serial.print("\n");






  //input is in this format:  "y x camy camx\n"
  firstspace = input.indexOf(' ');
  secondspace = input.indexOf(' ', firstspace + 1);
  thirdspace = input.indexOf(' ', secondspace + 1);
  y = input.substring(0, firstspace);
  x = input.substring(firstspace + 1, secondspace);
  camx = input.substring(secondspace + 1, thirdspace);
  camy = input.substring(thirdspace + 1); //takes rest of string


  //is this horribly inelegant? Probably, but nobody is going to see this code but me, probably. faster than constantly recalculating indices.

  if ( y == "c"){
    centerCamera(); 
  }
  else{
  xpos = x.toInt();
  ypos = y.toInt();
  camxpos = camx.toInt();
  camypos = camy.toInt();

  //should probably add some sanity checks to make sure we're not getting garbage.
  //END SERIAL STUFF

  ypos = ypos - 2042; // center y around 0
  int ymag = abs(ypos);

  camxpos = camxpos - 2042;
  camypos = camypos - 2042;
  int camxmag = abs(camxpos);
  int camymag = abs(camypos);


  int xangle = map(xpos, 0, 4084, 0, 180); //map to servo angle range

  int yscale = map(ymag, 0, 2042, 0, 255); //map to PWM range

  int camxscale = (camxmag == 0) ? 0 : map(camxmag, 0, 2042, 120, 255);
  int camyscale = (camymag == 0) ? 0 : map(camymag, 0, 2042, 120, 255);


  analogWrite(pinPWM, yscale/2 );
  digitalWrite(pinDir, (ypos > 0) ? LOW : HIGH); //gets rid of annoying big if else statements

  if (camxscale != 0) {
    analogWrite(pinCamxPWM, camxscale / 4); //full speed might be too fast, we can adjust if need be
    digitalWrite(pinCamxDir, (camxpos > 0) ? LOW : HIGH);  //shouldn't need to take 0 as a separate case now that we don't have noise
    lastxspeed = camxscale / 4;
    lastxdir = ((camxpos > 0) ? 0 : 1);
  }
  else //brake
  {
    analogWrite(pinCamxPWM, lastxspeed / BRAKESPEED);
    digitalWrite(pinCamxDir, !(lastxdir));
    lastxspeed = 0;
    lastxdir = 0;
  }

  if (camyscale != 0) {
    analogWrite(pinCamyPWM, camyscale / 4);
    digitalWrite(pinCamyDir, (camypos > 0) ? LOW : HIGH);
    lastyspeed = camyscale / 4;
    lastydir = ((camypos > 0) ? 0 : 1);
  }
  else
  {
    analogWrite(pinCamyPWM, lastyspeed / BRAKESPEED);
    digitalWrite(pinCamyDir, !(lastydir));
    lastyspeed = 0;
    lastydir = 0;
  }




  steer.write(xangle);


  }
  delay(15); //Note: in the arduino example they use 15 ms/ servo degree to get 'smooth' motion. This is trying the same delay
}
