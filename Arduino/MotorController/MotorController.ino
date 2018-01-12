
#include <PID_v1.h>
#include <Encoder.h> 
#include "Motors.h"

Motor M[4] = 
{ 
  {1,M1EnA,M1EnB},
  {2,M2EnA,M2EnB},
  {3,M3EnA,M3EnB},
  {4,M4EnA,M4EnB}
}; 
              
unsigned long lastMilli = 0;
unsigned char incomingByte[2];

void setup() {
  Serial.begin(9600);
  incomingByte[0]=0;
  incomingByte[1]=0;
  for(int j=0;j<4;j++) {
    M[j].run(FORWARD);
  }
}

void loop() {
  
  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();

    if (Serial.available()>0) {   
      incomingByte[0]=Serial.read();
      incomingByte[1]=Serial.read(); 
    } 
    
    int MotorNum = (incomingByte[0]>>5);
    int Dir = (incomingByte[0]>>3)&0x03;
    int Speed = incomingByte[1];

    Serial.print(incomingByte[0],HEX); Serial.print("  ");
    Serial.print(M[MotorNum-1].IN1pin); Serial.print("  ");    
    Serial.print(Dir); Serial.print("  ");
    Serial.print(Speed); Serial.println("  ");

    if (Dir==STOP) 
      M[MotorNum-1].run(STOP);
    else if(Dir==FORWARD) 
      M[MotorNum-1].run(FORWARD);
    else if(Dir==BACKWARD) 
      M[MotorNum-1].run(BACKWARD);

    M[MotorNum-1].Setpoint = Speed;

    for(int j=0;j<4;j++) {
      M[j].updatePID();
    }
    
  } 
}





