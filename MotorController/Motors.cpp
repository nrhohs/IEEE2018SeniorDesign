#include "Arduino.h"
#include "Motors.h"




Motor::Motor(void){
  PWMpin=0;
  IN1pin=0;
  IN2pin=0;
}

Motor::motorSetup(uint8_t motorID) {
  if(motorID == 1){
    PWMpin = M1PWMPIN;
    IN1pin = M1FPIN;
    IN2pin = M1RPIN;
  }
  else if(motorID == 2){
    PWMpin = M2PWMPIN;
    IN1pin = M2FPIN;
    IN2pin = M2RPIN;
  }  
  else if(motorID == 3){
    PWMpin = M3PWMPIN;
    IN1pin = M3FPIN;
    IN2pin = M3RPIN;
  }
  else if(motorID == 4){
    PWMpin = M4PWMPIN;
    IN1pin = M4FPIN;
    IN2pin = M4RPIN;
  }
  else if(motorID == 5){
    PWMpin = M5PWMPIN;
    IN1pin = M5FPIN;
    IN2pin = M5RPIN;
  }
  else if(motorID == 6){
    PWMpin = M6PWMPIN;
    IN1pin = M6FPIN;
    IN2pin = M6RPIN;
  }
  pinMode(PWMpin, OUTPUT);
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
}

Motor::run(uint8_t cmd){
  if(cmd == FORWARD){
    digitalWrite(IN1pin,HIGH);
    digitalWrite(IN2pin,LOW);
  }
  else if(cmd == BACKWARD){
    digitalWrite(IN1pin,LOW);
    digitalWrite(IN2pin,HIGH);
  }  
  else if(cmd == STOP){
    digitalWrite(IN2pin,LOW);
    digitalWrite(IN2pin,LOW);
  }  
}

Motor::setSpeed(uint8_t pwm){
  analogWrite(PWMpin, pwm);
}

