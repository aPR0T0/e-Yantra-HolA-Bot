const int dirPin_1 = 8;
const int stepPin_1 = 6;
const int dirPin_2 = 9;
const int stepPin_2 = 7;
const int dirPin_3 = 0;
const int stepPin_3 = 0;
const int stepsPerRevolution = 300;

void setup() {
  pinMode(stepPin_1, OUTPUT);
  pinMode(dirPin_1, OUTPUT);
  pinMode(stepPin_2, OUTPUT);
  pinMode(dirPin_2, OUTPUT);
  pinMode(stepPin_3, OUTPUT);
  pinMode(dirPin_3, OUTPUT);

}

void loop() {
  // Set motor direction clockwise
  digitalWrite(dirPin_1, HIGH);

  digitalWrite(dirPin_2, LOW);
  digitalWrite(dirPin_3, HIGH);
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin_2 , HIGH);
    digitalWrite(stepPin_3 , HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin_2, LOW);
    digitalWrite(stepPin_3, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);
  
  digitalWrite(dirPin_1, HIGH);
  digitalWrite(dirPin_3, LOW);
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin_1 , HIGH);
    digitalWrite(stepPin_3 , HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin_1, LOW);
    digitalWrite(stepPin_3, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);

  digitalWrite(dirPin_1, LOW);
  digitalWrite(dirPin_2, HIGH);
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    digitalWrite(stepPin_1 , HIGH);
    digitalWrite(stepPin_2 , HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepPin_1, LOW);
    digitalWrite(stepPin_2, LOW);
    delayMicroseconds(2000);
  }
  delay(1000);

}
