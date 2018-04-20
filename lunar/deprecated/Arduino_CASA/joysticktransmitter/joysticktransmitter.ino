/*

 */

// constants won't change. They're used here to 
// set pin numbers:
const int joystickX = A0;    //j1.2   // the number of the joystick X-axis analog
const int joystickY =  A1;   //j3.26  // the number of the joystick Y-axis analog


// variables will change:
int joystickSelState = 0;      // variable for reading the joystick sel status
int joystickXState, joystickYState ; 

void setup() {


  
  Serial.begin(9600);  
  Serial.setTimeout(1000); 
}

void loop(){
  // read the analog value of joystick x axis
  joystickXState = analogRead(joystickX);
  // read the analog value of joystick y axis
  joystickYState = analogRead(joystickY);
  Serial.print(map(joystickYState,0,728,0,4084)); //this mapping is done due to an error of mine in writing the receiver code: it is easier to map the transmitter output than it is to rewrite the receiver to accept lower precision.
  Serial.print(" "); 
  Serial.print(map(joystickXState,0,728,0,4084)); 
  Serial.print("\n"); 
 // while(Serial.available() == 0); //spin on command received response, but we don't want to wait forever 
  String response = Serial.readStringUntil('\n'); 
}
