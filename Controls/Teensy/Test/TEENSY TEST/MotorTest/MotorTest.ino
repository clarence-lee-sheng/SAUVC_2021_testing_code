#include <Servo.h>

#define MOTORSIZE 5
#define M1_PIN 23
#define M2_PIN 10
#define M3_PIN 14
#define M4_PIN 22
#define M5_PIN 15


struct Motor 
{
   int pinNo;
   int armingFreq; 
   Servo esc; 
};

Motor motors[MOTORSIZE];
int motorPins [MOTORSIZE] = {M1_PIN,M2_PIN,M3_PIN,M4_PIN,M5_PIN};
int armingFreq[MOTORSIZE] = {1480,1480,1480,1480,1480};
Servo esc;

void setup() 
{
  Serial.begin(9600);
  armAll();
}

void loop() 
{
  test();
}
void armAll()
{
  Serial.println("Arming Motors");
  for(int i = 0 ; i<MOTORSIZE ; i++)
  {
    motors[i].pinNo = motorPins[i];
    motors[i].armingFreq = armingFreq[i];
    motors[i].esc.attach(motorPins[i]);

    motors[i].esc.writeMicroseconds(motors[i].armingFreq);
    ///BLOCKING CODE 
    delay(3000);
    ///BLOCKING CODE 
    Serial.print(motors[i].pinNo);Serial.print(" ");Serial.println(motors[i].armingFreq);
  }
  Serial.println("MOTORS ARMED !!!");
}
void test()
{
   Serial.println("spinning cw");
   for(int i = 1500 ; i<1900 ; i+=10 )
   {
     Serial.println(i);
     motors[0].esc.writeMicroseconds(i);delay(500);
     motors[1].esc.writeMicroseconds(i);delay(500);
     motors[2].esc.writeMicroseconds(i);delay(500);
     motors[3].esc.writeMicroseconds(i);delay(500);
     motors[4].esc.writeMicroseconds(i);delay(500);
   }
   for(int i = 1899 ; i>1500 ; i-=10)
   {
    Serial.println(i);
    motors[0].esc.writeMicroseconds(i);delay(500);
    motors[1].esc.writeMicroseconds(i);delay(500);
    motors[2].esc.writeMicroseconds(i);delay(500);
    motors[3].esc.writeMicroseconds(i);delay(500);
    motors[4].esc.writeMicroseconds(i);delay(500);
   }
   //esc.write(91);delay(5000);

   for(int i = 1500 ; i>1100 ; i-=10)
   {
     Serial.println(i);
     motors[0].esc.writeMicroseconds(i);delay(500);
     motors[1].esc.writeMicroseconds(i);delay(500);
     motors[2].esc.writeMicroseconds(i);delay(500);
     motors[3].esc.writeMicroseconds(i);delay(500);
     motors[4].esc.writeMicroseconds(i);delay(500);
   }
   for(int i = 1101 ; i>1500 ; i+=10)
   {
    Serial.println(i);
    motors[0].esc.writeMicroseconds(i);delay(500);
    motors[1].esc.writeMicroseconds(i);delay(500);
    motors[2].esc.writeMicroseconds(i);delay(500);
    motors[3].esc.writeMicroseconds(i);delay(500);
    motors[4].esc.writeMicroseconds(i);delay(500);
   }
}
