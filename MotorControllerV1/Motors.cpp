#include "Arduino.h"
#include "Motors.h"



Motor::Motor(uint8_t motorID, uint8_t pin1, uint8_t pin2)
    :MPID(&Input,&Output,&Setpoint,Kp,Ki,Kd,DIRECT), MEncoder(pin1,pin2)
{
  PWMpin=0;
  IN1pin=0;
  IN2pin=0;
  prevCount=0;
  Setpoint=200; 
  MPID.SetMode(AUTOMATIC);
  MPID.SetOutputLimits(-255,255);

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
    digitalWrite(IN2pin,LOW);
    digitalWrite(IN1pin,HIGH);
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


void Motor::updatePID() {
  Position = MEncoder.read();
  Input = getRPM(Position);
  Serial.println(int(Input));  
  MPID.Compute();
  MPWM = constrain(int(MPWM+Output),0,255);
  setSpeed(MPWM);
}


double Motor::getRPM(long count) {
  double speed_act = ((count - prevCount)*(60*(1000/LOOPTIME)))/(1920.0);
  prevCount = count;
  return speed_act;
}


