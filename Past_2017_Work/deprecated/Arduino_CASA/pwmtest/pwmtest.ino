void setup() {
 pinMode(12, OUTPUT); 
 pinMode(5, OUTPUT); 

}

void loop() {
  float pwmsig = 0.1; 
  while(1){
  digitalWrite(5, HIGH); 
  analogWrite(12, pwmsig*255); 
  delay(1000); 
  pwmsig += 0.05; 
  }

}
