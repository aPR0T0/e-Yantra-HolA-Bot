const int dirPin = 9;
const int stepPin = 7;
const int enablePin = 11;
const int stepsPerRevolution = 200;
const int x1 = 450;

// Let's check the rpm and delay relation in NEMA-16 motors

// Max RPM is at 450 ms  consider this as highest
// Min RPM is at 2500 ms consider this as lowest at this point we can set the enable pin to high which will stop the motor

void setup()
{
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
}
void loop()
{
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);
  for(int x = 0; x < stepsPerRevolution; x++)
  {
    if(x1 > 2500){
      digitalWrite(enablePin, HIGH);
    }
    else{
      digitalWrite(stepPin,  HIGH);
      delayMicroseconds(x1);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(x1);
    }
  }
  delay(1000); 

}
