#include "Motors.h"

Motor M[4] = 
{ 
  {1,M1EnA,M1EnB},
  {2,M2EnA,M2EnB},
  {3,M3EnA,M3EnB},
  {4,M4EnA,M4EnB}
}; 
              
unsigned long lastMilli = 0;

void setup() {
  Serial.begin(9600);
  int i;
  for(i=0;i++;i<4) 
    M[i].run(FORWARD);
}

void loop() {
  int i, j;
  unsigned char incomingByte[2];
  if (Serial.available()>0) {
    incomingByte[i%2]=Serial.read();
    i++;
  }
  int MotorNum = incomingByte[0]>>5;
  int Dir = (incomingByte[0]>>3)&&0xFF;
  int Speed = incomingByte[1];
  if (Dir==STOP) 
    M[MotorNum].run(STOP);
  if(Dir==FORWARD)
    M[MotorNum].run(FORWARD);
  if(Dir==REVERSE)
    M[MotorNum].run(REVERSE);
  M[MotorNum].Setpoint = Speed; 
  Serial.print(MotorNum); Serial.print("  ");
  Serial.print(Dir); Serial.print("  ");
  Serial.print(Speed); Serial.println("  ");
  
  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
    for(j=0;j++;j<4)
      M[j].updatePID();
  }
}





