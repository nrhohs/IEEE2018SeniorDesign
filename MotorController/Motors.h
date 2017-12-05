#ifndef Motors_h
#define Motors_h

#include "Arduino.h"

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

#define FORWARD 1
#define BACKWARD 2
#define STOP 3


class Motor
{
 public:
  Motor(void);
  void motorSetup(uint8_t);
  void run(uint8_t);
  void setSpeed(uint8_t);

  private:
  uint8_t PWMpin, IN1pin, IN2pin;
  uint8_t motorID;
};

#endif


