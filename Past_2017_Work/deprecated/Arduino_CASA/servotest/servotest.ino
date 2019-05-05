#include <Servo.h>
Servo datvo; 

void setup() {
  datvo.attach(4); 
}

void loop() {
  datvo.write(90); 
  delay(1000);
  
  for(int i=90; i<180; i++) {
    datvo.write(i); 
    delay(20); 
  }

  datvo.write(90); 
  delay(1000);

  for(int i=90; i>0; i--) {
    datvo.write(i); 
    delay(20); 
  }

  
}
