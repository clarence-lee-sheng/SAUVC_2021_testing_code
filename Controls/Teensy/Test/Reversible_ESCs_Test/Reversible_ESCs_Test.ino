#include <Servo.h>

byte servoPin = 11;
Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(servoPin);
  
  delay(2000); // delay to allow the ESC to recognize the stopped signal
  arm(servo);

  
}

void loop() {
// Set signal value, which should be between 1100 and 1900
// 1100(max) - 1499(min) is clockwise, 1500 is stop, 1501(min) - 1900(max) is anti-clockwise

  servo.writeMicroseconds(1800); // Send signal to ESC.
  delay(800);
  servo.writeMicroseconds(1200); // Send signal to ESC.
  delay(800);
}

void arm(Servo m){
  Serial.print("Arming.");
  servo.write(89);
  delay(200);
  Serial.println(" Armed!");
}
