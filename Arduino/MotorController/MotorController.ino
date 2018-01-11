#include <PID_v1.h>
#include <Encoder.h>
#include "Motors.h"


#define LOOPTIME    100
#define NUMREADINGS 10
#define Kp          0.10
#define Ki          0.05
#define Kd          0.01
  
double Setpoint, Input, Output;
int M1PWM = 0;



Motor MR1;
Encoder M1Encoder(18,52);
PID M1PID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT);

unsigned long lastMilli = 0;
volatile long count = 0;
int RPM = 0;
int i=0;


void setup() {
  Serial.begin(9600);
  MR1.motorSetup(1);
  MR1.run(FORWARD);
  M1PID.SetMode(AUTOMATIC);
  M1PID.SetOutputLimits(-255,255);
  Setpoint = 200;
}

void loop() {
  uint8_t incomingByte[2];
  if (Serial.available() > 0) { // if any data available
    incomingByte[i%2] = Serial.read(); // read byte
    Serial.write(int(incomingByte[i%2])); // send it back  
    i++;
  }
  
  


  
  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
    Input = getRPM(M1Encoder.read());
    M1PID.Compute();
    M1PWM = constrain(int(M1PWM+Output),0,255);
    MR1.setSpeed(M1PWM);
  } 
}

double getRPM(long count) {
  static long countAnt = 0;
  double speed_act = ((count - countAnt)*(60*(1000/LOOPTIME)))/(1920.0);
  countAnt = count;
  return speed_act;
}



