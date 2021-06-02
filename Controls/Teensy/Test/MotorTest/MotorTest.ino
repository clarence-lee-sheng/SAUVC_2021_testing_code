#include  <Servo.h>

Servo motor; 
byte servoPin = 9; // signal pin for the ESC.
void setup() {
 pinMode(LED_BUILTIN , OUTPUT);
 digitalWrite(LED_BUILTIN , HIGH);
 delay(1000);
 Serial.begin(9600);
 
 motor.attach(servoPin,1000,2000);
 digitalWrite(LED_BUILTIN , LOW );
 delay(1000);
 motor.writeMicroseconds(1500);
 digitalWrite(LED_BUILTIN , HIGH);
 Serial.println("Stop Command Sent : "); 
 Serial.println("Delay : "); 
 delay(2000);

 digitalWrite(LED_BUILTIN , LOW);
 delay(1000);

}

void loop() {
  // put your main code here, to run repeatedly:
   Serial.println("Running code :");
   digitalWrite(LED_BUILTIN , HIGH); 
   motor.writeMicroseconds(2000);
   digitalWrite(LED_BUILTIN , LOW);
}
