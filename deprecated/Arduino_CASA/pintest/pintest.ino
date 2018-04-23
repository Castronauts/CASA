#define PIN 21

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN, OUTPUT); 
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(PIN, LOW); 
  delay(3000); 
  digitalWrite(PIN, HIGH); 
  delay(3000); 
  
}
