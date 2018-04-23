#include <Servo.h>

  const int pinDir = 5; 
  const int pinPWM = 12; 
  const int pinServo = 7; 
  const int pinCamx = 4; 
  const int pinCamy = 10; 
  Servo steer; 
  Servo camxServo;
  Servo camyServo;
  int campos = 450; 
  int camrestx = 90; 
  int camresty = 90; 


  
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1000); //don't wait longer than the joystick sends commands, should prevent overflow
  pinMode(pinDir, OUTPUT); 
  pinMode(pinPWM, OUTPUT); 
  pinMode(pinServo, OUTPUT); 
  steer.attach(pinServo); 
  camxServo.attach(pinCamx); 
  camyServo.attach(pinCamy);
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


  
//  while (Serial.available() == 0); // spin on serial input. NOTE THAT THIS MEANS WE DO THE LAST COMMAND.
  //can we then check if Serial available is non-zero? 
WAIT_SERIAL:
  input = Serial.readStringUntil('\n'); //this should in theory read until \n or timeout. HOPEFULLY timeout is long enough to not eat data 
  if (input == ""){ //no command received, exited on timeout
      analogWrite(pinPWM, 0); //stop running away
      digitalWrite(pinDir, HIGH); 
      goto WAIT_SERIAL; //fuck you, code elitists 
  }
  Serial.print("r\n"); //command acknowledgement. We force synchronicity to prevent buffer overflows and unintended behavior
 /* Serial.print("Input is "); 
  Serial.print(input); 
  Serial.println(); */

 

  //input is in this format:  "y x camy camx\n"
  firstspace = input.indexOf(' '); 
  secondspace = input.indexOf(' ',firstspace+1); 
  thirdspace = input.indexOf(' ',secondspace+1);
  y = input.substring(0, firstspace);
  x = input.substring(firstspace+1, secondspace); 
  camy = input.substring(secondspace+1, thirdspace); 
  camx = input.substring(thirdspace+1); //takes rest of string 

  //is this horribly inelegant? Probably, but nobody is going to see this code but me, probably. faster than constantly recalculating indices. 

  xpos = x.toInt();
  ypos = y.toInt(); 
  camxpos = camx.toInt();
  camypos = camy.toInt(); 

  //should probably add some sanity checks to make sure we're not getting garbage. 
  //END SERIAL STUFF

  /*analogWrite(pinPWM, ypos); 
  digitalWrite(pinDir, LOW); //LOW IS FORWARD */

  ypos = ypos - 2030; // center y around 0 
  int ymag = abs(ypos); 

  /*if (ymag < 700) { //we shouldn't need to filter the digital joystick 
    ymag = 0; 
    ypos = 0; 
  }*/
  
  int camspeed = 15; 
  int xangle = map(xpos, 0, 4084, 0, 180); //map to servo angle range
 /* int camxangle = map(camxpos, 0, 4084, -15, 15); //these are really speed numbers, not angle numbers. NOTE: This is the max amount of change we can write to the servo per iteration.
  int camyangle = map(camypos, 0, 4084, -15, 15); */
  int camxangle; 
  int camyangle;
  if(camxpos > 2042) 
    camxangle = camspeed; 
  else if(camxpos < 2042)
    camxangle = -camspeed; 
  else 
    camxangle = 0; 

  if(camypos > 2042) 
    camyangle = camspeed; 
  else if(camypos < 2042) 
    camyangle = -camspeed; 
  else 
    camyangle = 0; 
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

    //camrestx and camresty start at 90 degrees. 
    camrestx+=camxangle; 
    camresty+=camyangle; 
    camrestx = constrain(camrestx,0,180); //this is ncessary or the servos lag 
    camresty = constrain(camresty,0,180); //Servos are very jerky, probably because they are being moved in position too fast. 
    steer.write(xangle); 
    camxServo.write(camrestx); 
    camyServo.write(camresty); 
    

     
  delay(1); //Note: in the arduino example they use 15 ms/ servo degree to get 'smooth' motion. This is trying the same delay
}
