
#include<Servo.h>

Servo esc;

// Currently attached pins = 23,22,21,20,10 

void setup()
{
  esc.attach(23);
/*
As you can see - I am not arming the ESC here.
For some reason, the ESC is now on a persistently armed mode now.
Upon powering, the ESC automatically arms.
Given sparse documentation for the ESC, I am not examining this behavior change.

However, if your ESC refuses to arm, see the first post in this thread.
*/

//Again - this is useful only during the testing part. For autonomous running, it is pointless.

  int i=0;
  Serial.begin(9600);
}

void loop()
{
  int i=0;
  // Start at Servo Degree 70 and go till just above neutral.
  // Note, Neutral is at 94 for this ESC.
  // This will run it in one direction.
  for(i=80;i<95;i++)
  {
    esc.write(i);
    delay(500);
    Serial.println(i);
  }
  
  Serial.println("Armed");
//Run for 2 seconds at moderately high speed (70).
  esc.write(70); delay(2000);
// Apply Brakes for 40 millisecs.
  esc.write(100); delay(40);
// Apply throttle in opposite direction for 2 seconds.
  esc.write(110); delay(2000);
// Bring down throttle to neutral.
  for(i=110;i>90;i--)
  {
    esc.write(i);
    delay(200);
  }
  Serial.println("disarmed");
// Apply brakes again - to change directions for 40 millisecs.
  esc.write(70); delay(40);
}
