#ifndef Motors_h
#define Motors_h

#include "Arduino.h"
#include <PID_v1.h>
#include <Encoder.h>


//Define Motor Pin Numbers
#define M1PWMPIN   8
#define M1FPIN     22
#define M1RPIN     23
#define M2PWMPIN   9
#define M2FPIN     24
#define M2RPIN     25
#define M3PWMPIN   10
#define M3FPIN     26
#define M3RPIN     27
#define M4PWMPIN   11
#define M4FPIN     28
#define M4RPIN     29
#define M5PWMPIN   12
#define M5FPIN     30
#define M5RPIN     31
#define M6PWMPIN   13
#define M6FPIN     32
#define M6RPIN     33

#define M1EnA      18
#define M1EnB      50
#define M2EnA      19
#define M2EnB      51
#define M3EnA      20
#define M3EnB      52
#define M4EnA      21
#define M4EnB      53
#define M5EnA      18
#define M5EnB      48
#define M6EnA      18
#define M6EnB      49

#define FORWARD 1
#define BACKWARD 2
#define STOP 0

#define LOOPTIME    100

#define Kp          0.10
#define Ki          0.05
#define Kd          0.01

class Motor
{
 public:
  Motor(uint8_t MotorID,uint8_t pin1,uint8_t pin2);
  run(uint8_t);
  setSpeed(uint8_t);
  Encoder MEncoder;
  PID MPID;
  void updatePID();
  double  Setpoint;

  private:
  uint8_t PWMpin, IN1pin, IN2pin, MPWM;
  uint8_t motorID;
  double  Input, Output;
  long Position, prevCount; 
  double getRPM(long);  
};

#endif


