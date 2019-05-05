#include <Servo.h>

  const int pinDir = 5; 
  const int pinPWM = 12; 
  const int pinServo = 7; 
  const int pinCam = 4; 
  Servo steer; 
  Servo cam; 
  int campos = 450; 

  
void setup() {
  Serial.begin(9600);
 // Serial.setTimeout(50); //don't wait longer than the joystick sends commands, should prevent overflow
  pinMode(pinDir, OUTPUT); 
  pinMode(pinPWM, OUTPUT); 
  pinMode(pinServo, OUTPUT); 
  steer.attach(pinServo); 
  cam.attach(pinCam); 
}

void loop() {
  int xpos;
  int ypos;
  //SERIAL STUFF HERE
  //first we suck up the whole line
  String input;
  String x;
  String y;

  while (Serial.available() == 0); // spin on serial input. NOTE THAT THIS MEANS WE DO THE LAST COMMAND.
  input = Serial.readStringUntil('\n');
  Serial.print("r\n"); //command acknowledgement. We force synchronicity to prevent buffer overflows and unintended behavior
 /* Serial.print("Input is "); 
  Serial.print(input); 
  Serial.println(); */

 

  
  y = input.substring(0, input.indexOf(' '));
  x = input.substring(input.indexOf(' ') + 1); //reads till end 

  xpos = x.toInt();
  ypos = y.toInt(); 

  //should probably add some sanity checks to make sure we're not getting garbage. 
  //END SERIAL STUFF

  /*analogWrite(pinPWM, ypos); 
  digitalWrite(pinDir, LOW); //LOW IS FORWARD */

  ypos = ypos - 2030; // center y around 0 
  int ymag = abs(ypos); 

  if (ymag < 700) {
    ymag = 0; 
    ypos = 0; 
  }
  

  int xangle = map(xpos, 0, 4050, 0, 180); //map to servo angle range
  int yscale = map(ymag, 0, 2042, 0, 255); //map to PWM range

     if(ypos > 0) //forward
      {
      analogWrite(pinPWM,yscale/4); //right now, 25% is the highest we want them to be able to drive at (since the rover goes so fast at 100%) 
      digitalWrite(pinDir, LOW); 
      }
     else if(ypos < 0) //backwards
     {
      analogWrite(pinPWM, yscale/4); //right now, 25% is the highest we want them to be able to drive at (since the rover goes so fast at 100%) 
      digitalWrite(pinDir, HIGH); //not sure if this is actually backwards, might have to remap after tuning. 
     } //MUST WRITE PINS LOW, NOT JUST HIGH!!!!! 
     else{
      analogWrite(pinPWM, 0); 
      digitalWrite(pinDir, HIGH); 
     } 

    //now we deal with x coordinates
    if(xangle > 140) 
      xangle = 140; 
    if(xangle < 40) 
      xangle = 40; 
    if(abs(xangle-90) < 30)
      xangle = 90; //stop small oscillations around the center caused by joystick noise
    steer.write(xangle); 
    cam.write(180-xangle); 

     
  delay(20); // not sure how necessary this is, might even work better without it */ 
}
