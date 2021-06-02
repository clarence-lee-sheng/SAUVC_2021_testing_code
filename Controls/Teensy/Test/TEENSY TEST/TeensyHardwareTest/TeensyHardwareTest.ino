
int led = 13;
bool state = true;
// the setup routine runs once when you press reset:
void setup() 
{
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() 
{
  setState(state);
  delay(1000);               // wait for a second
}

void setState(bool _state)
{
  Serial.println(_state);
  if(_state)
  {
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    state = false; 
  }
  else
  {
    digitalWrite(led, LOW);  // turn the LED on (HIGH is the voltage level)
    state = true; 
  }
}
