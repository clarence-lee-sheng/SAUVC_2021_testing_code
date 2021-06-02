#include <Servo.h>

#define MOTORSIZE 5 // Change this if this physical motor count increases
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
int generalArmingFreq = 1480;
Servo esc;

void setup() 
{
  Serial.begin(9600);
  armAll();
}

void loop() 
{
  //test();
  testSingle(motors[1]);
}

//============================INIT CODE ==================================
void armAll()
{
  Serial.println("Arming Motors");
  delay(5000);
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

///============================COMMS CODE ==================================
void serialEvent()
{
  //!WARNING : TEMP CODE REPLACE WITH LOOK UP FUCNTION FOR DYNAMIC COMMS METHOD
  String readValue = Serial.readString(); 
  //readValue.flush();
  int value = readValue.toInt(); 
  Serial.print("ReadValue : ");Serial.println(readValue);
  Serial.print("ActualValue : ");Serial.println(value);
}
//TODO : Add ROSSERIAL CODE HERE

///============================MOTOR CONTROL CODE ==================================
void motorSetSpeedAllPercentage(int _speedPercentage [])
{
  //TAKE NOTE : MAKE SURE THAT _speedPercentage ARRAY SIZE IS EQUALS TO MOTORSIZE
  for(int i =0 ;i<MOTORSIZE ; i++)
  {
    motors[i].esc.writeMicroseconds(freqConversion(_speedPercentage[i]));  
    Serial.print("Motor Speed change : Motor : "); Serial.print(i); Serial.print(" speedPercentage : ");Serial.println(_speedPercentage[i]);
  }
}

void motorSetSpeedFreq(Motor _motor,int _speedFreq)
{
  //TODO:SCALE THIS UP TO 4 MOTORS FOR EASE OF CONTROL
  _motor.esc.writeMicroseconds(_speedFreq);
  Serial.print("Motor Speed change : ");Serial.println(_speedFreq);
}
void motorSetSpeedPercentage(Motor _motor,int _speedPercentage)
{
  //TODO:SCALE THIS UP TO 4 MOTORS FOR EASE OF CONTROL 
  _motor.esc.writeMicroseconds(freqConversion(_speedPercentage));
  Serial.print("Motor Speed change : ");Serial.println(_speedPercentage);
}
///============================COMPUTATION CODE ==================================
int freqConversion(int speedPercentage)
{
  if(speedPercentage >= 0)
  {
    //CW code : 
    int freq = map(speedPercentage,0,100,generalArmingFreq,1900 );
    return freq;
  }
  else if(speedPercentage <=0 )
  {
    //ACW code : 
    int freq = map(-speedPercentage,0,100,1100,generalArmingFreq);
    return freq;
  }
  else
  {
    //default code 
    Serial.print("freq :Invalid input : speedPercentage : ");Serial.println(speedPercentage);
  }
}

///============================TESTING CODE ==================================
void testSingle(Motor motor)
{
  Serial.println("spinning cw");
  int delayTime = 1000;
   for(int i =0 ; i<100 ; i+=10 ) 
   {
     Serial.println(i);
     motorSetSpeedPercentage(motor , i);
     delay(delayTime);
   }
   for(int i =100  ; i>0 ; i-=10 ) 
   {
    Serial.println(i);
    motorSetSpeedPercentage(motor , i);
    delay(delayTime);
   }
}
void test()
{
   Serial.println("spinning cw");
   int delayTime = 1000;
   for(int i =0  ; i<100 ; i+=1 ) 
   {
     Serial.println(i);
     int speedPercentage[] = {i,i,i,i,i};
     motorSetSpeedAllPercentage(speedPercentage);

     delay(delayTime);
   }
   for(int i =100  ; i>0 ; i-=1 ) 
   {
     Serial.println(i);
     int speedPercentage[] = {i,i,i,i,i}; 
     motorSetSpeedAllPercentage(speedPercentage);
     
     delay(delayTime);
   }
   for(int i =0  ; i>-100 ; i-=1 ) 
   {
     Serial.println(i);
     int speedPercentage[] = {i,i,i,i,i};
     motorSetSpeedAllPercentage(speedPercentage);

     delay(delayTime);
   }
   for(int i =-100  ; i<0 ; i+=1 ) 
   {
     Serial.println(i);
     int speedPercentage[] = {i,i,i,i,i}; 
     motorSetSpeedAllPercentage(speedPercentage);
     
     delay(delayTime);
   }
}
