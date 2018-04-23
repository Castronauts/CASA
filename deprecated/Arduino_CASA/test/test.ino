/*
ECEN2830 motor speed control example (left wheel only)
*/

// define pins
const int pinON = 6;         // connect pin 6 to ON/OFF switch, active HIGH
const int pinCW_Left = 7;    // connect pin 7 to clock-wise PMOS gate
const int pinCC_Left = 8;    // connect pin 8 to counter-clock-wise PMOS gate
const int pinCW_Right = 11; 
const int pinCC_Right = 12; 
const int pinSpeed_Left = 9; // connect pin 9 to speed reference
const int pinSpeed_Right = 10; 

volatile int lcount = 0; 
volatile int rcount = 0; 
// setup pins and initial values
void setup() {
  noInterrupts(); 
  attachInterrupt(digitalPinToInterrupt(2), EncLeft, RISING); 
  attachInterrupt(digitalPinToInterrupt(3), EncRight, RISING); 
  pinMode(pinON,INPUT);
  pinMode(pinCW_Left,OUTPUT);
  pinMode(pinCC_Left,OUTPUT);
  pinMode(pinCW_Right,OUTPUT);
  pinMode(pinCC_Right,OUTPUT);
  pinMode(pinSpeed_Right, OUTPUT); 
  pinMode(pinSpeed_Left,OUTPUT);
  pinMode(13,OUTPUT);             // on-board LED
  digitalWrite(pinCW_Left,LOW);   // stop clockwise
  digitalWrite(pinCC_Left,LOW);   // stop counter-clockwise
  analogWrite(pinSpeed_Left,100); // set speed reference, duty-cycle = 100/255
  analogWrite(pinSpeed_Right,100); 
  interrupts(); 
}

void EncLeft(){
     lcount++;  //note, there could be a concurrency issue but this interrupt shoudl be several orders of magnitude faster than the encoder pulses. If an interrupt interrupts this, could corrupt stuff.
}

void EncRight(){
    rcount++; 
}

void TurnRight(int degrees){
      float rots = degrees/360; //only do FP division once. convert degrees to rotations
      int count = int(12.0*rots); //64 encoder pulses per rotation
      lcount = 0; 
      rcount = 0; //we'll only use rcount for this one, but set both for safety
      digitalWrite(pinCW_Right,HIGH);  // start turning right 
      digitalWrite(pinCC_Right,LOW); 
      while(rcount<=count); //We're spinning here which is power intensive but hopefully ok. 
  
}

void TurnLeft(int degrees){
      float rots = degrees/360; //only do FP division once. convert degrees to rotations
      int count = int(12.0*rots); //64 encoder pulses per rotation
      lcount = 0; 
      rcount = 0; //we'll only use rcount for this one, but set both for safety
      digitalWrite(pinCW_Left,HIGH);
      digitalWrite(pinCC_Left,LOW); 
      digitalWrite(pinCC_Right,HIGH); 
      digitalWrite(pinCW_Right,LOW); 
      while(lcount<=count); //We're spinning here which is power intensive but hopefully ok. 
  
}

void GoForward(int degrees){
      float rots = degrees/360; //only do FP division once. convert degrees to rotations
      int count = int(12.0*rots); //64 encoder pulses per rotation
      lcount = 0; 
      rcount = 0; //we'll only use rcount for this one, but set both for safety
      digitalWrite(pinCW_Left,LOW);
      digitalWrite(pinCC_Left,HIGH);

      while(lcount<=count); //We're spinning here which is power intensive but hopefully ok. 
  
}

void GoBackward(int degrees){
      float rots = degrees/360; //only do FP division once. convert degrees to rotations
      int count = int(12.0*rots); //64 encoder pulses per rotation
      lcount = 0; 
      rcount = 0; //we'll only use rcount for this one, but set both for safety
      digitalWrite(pinCC_Left,LOW);
      digitalWrite(pinCW_Left,HIGH);
      digitalWrite(pinCC_Right,HIGH);  
      digitalWrite(pinCW_Right,LOW);
      while(lcount<=count); //We're spinning here which is power intensive but hopefully ok. 
  
}

void loop() {
  digitalWrite(13,LOW);                     // turn LED off
  do {} while (digitalRead(pinON) == LOW);  // wait for ON switch
  TurnRight(360); 
  delay(10000);
  TurnLeft(360); 
}
