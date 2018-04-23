const int pinPWM = 3; 
const int pinDir = 12; 

void setup() {
   Serial.begin(9600);
   Serial.setTimeout(10000); 
  pinMode(pinDir, OUTPUT); 
  pinMode(pinPWM, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:
  String input; 
  int pos; 
  
  input = Serial.readStringUntil('\n'); 
  Serial.print(input); //also prints in timeout
  pos = input.toInt(); 

  //it is expecting a number from 0-255 
  
  analogWrite(pinPWM, pos); 
  digitalWrite(pinDir, LOW); 
}
