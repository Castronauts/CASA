/****************************************
 ********** Rover Testing Code **********
 ***************************************/

#include <Servo.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>

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

#define motorTestSpeed 12
#define camTestSpeed 255
#define motorTestTime 1000


const int pinDir = 5; // Pin for motor direction
const int pinPWM = 10; // Pin for motor speed
const int pinServo = 7; // Pin for steering servo
const int pinCamxPWM = 3; //this is A
const int pinCamxDir = 12;
const int pinCamyPWM = 11; //this is B
const int pinCamyDir = 13;
float x_angle; // Angle of tilt parallel to long edge
float y_angle; // Angle of tilt parallel to short edge
float pitch;
float roll;

Servo steer;
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

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


void calcOrientation(float x, float y, float z)
{
  pitch = atan2(x, sqrt((y * y) + (z * z)));
  roll = atan2(y, sqrt((x * x) + (z * z)));
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;
  return;
}

void DCtest(void)
{
  // Test DC motor one way
  Serial.println("pinDir = LOW, pinPWM at defined motorTestSpeed. Begin motorTestTime period test.");
  delay(100);
  digitalWrite(pinDir, LOW);
  analogWrite(pinPWM, motorTestSpeed);
  delay(motorTestTime);
  analogWrite(pinPWM, 0);

  // Test DC motor the other way
  Serial.println("pinDir = HIGH, pinPWM at motorTestSpeed. Begin motorTestTime period test.");
  delay(100);
  digitalWrite(pinDir, HIGH);
  analogWrite(pinPWM, motorTestSpeed);
  delay(motorTestTime);
  analogWrite(pinPWM, 0);
}

void steerTest(void)
{
  // Test steering servo
  Serial.println("Steering servo, full range-of-motion. Begin test.");
  delay(500);
  Serial.println("Writing position '0' to steer.");
  delay(250);
  steer.write(0);
  delay(500);
  Serial.println("writing position '180' to steer.");
  delay(250);
  steer.write(180);
  delay(500);
}

void CamX(void)
{
  // Test camera motor in x-axis
  Serial.println("Testing camera x-movement, direction LOW, speed defined camTestSpeed.");
  delay(100);
  digitalWrite(pinCamxDir, LOW);
  analogWrite(pinCamxPWM, camTestSpeed);
  delay(750);
  analogWrite(pinCamxPWM, 0);
  delay(100);

  Serial.println("Direction HIGH, speed defined camTestSpeed.");
  delay(100);
  digitalWrite(pinCamxDir, HIGH);
  analogWrite(pinCamxPWM, camTestSpeed);
  delay(750);
  analogWrite(pinCamxPWM, 0);
  delay(100);
}

void CamY(void)
{
  // Test camera motor in y-axis
  Serial.println("Testing camera y-movement, direction LOW, speed camTestSpeed.");
  delay(100);
  digitalWrite(pinCamyDir, LOW);
  analogWrite(pinCamyPWM, camTestSpeed);
  delay(750);
  analogWrite(pinCamyPWM, 0);
  delay(100);

  Serial.println("Direction HIGH, speed camTestSpeed.");
  delay(100);
  digitalWrite(pinCamyDir, HIGH);
  analogWrite(pinCamyPWM, camTestSpeed);
  delay(750);
  analogWrite(pinCamyPWM, 0);
  delay(100);
}

void printIMUdata(void)
{
  Serial.print(x_angle); //send IMU data back to operator
  Serial.print(" ");
  Serial.print(y_angle);
  Serial.print("\n");
}

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(1000); //don't wait longer than the joystick sends commands, should prevent overflow
  
  bool success = lsm.begin();
  if(success != 1)
    Serial.println("failed to initalize IMU"); 
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

  pinMode(pinDir, OUTPUT);
  pinMode(pinPWM, OUTPUT);
  pinMode(pinServo, OUTPUT);
  pinMode(pinCamxDir, OUTPUT); 
  pinMode(pinCamyDir, OUTPUT); 
  steer.attach(pinServo);
}

void loop()
{

  readGyro();
  printIMUdata();
  //DCtest();
  //readGyro();
  //printIMUdata();
   steerTest();
  //readGyro();
  //printIMUdata();
   CamX();
  //readGyro();
  //printIMUdata();
   CamY();
  //  centerCamera();
  delay(2000); 
}

