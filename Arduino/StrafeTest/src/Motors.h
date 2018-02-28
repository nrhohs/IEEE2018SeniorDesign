#ifndef Motors_h
#define Motors_h


#include "Arduino.h"
#include <PID_v1.h>
#include <Encoder.h>


#define FORWARD    1
#define BACKWARD   2
#define STOP       3

#define LOOPTIME    100

#define Kp          0.3
#define Ki          0.075
#define Kd          0.035

class Motor
{
 public:
  Motor(uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);
  void run(uint8_t);
  void setSpeed(uint8_t);
  long getPosition();
  void resetPosition();
  Encoder MEncoder;
  PID MPID;
  void updatePID();
  double  Setpoint;

  private:
  uint8_t PWMpin, IN1pin, IN2pin, MPWM;  
  double  Input, Output;
  long Position, prevCount; 
  double getRPM(long);  
};



#endif


