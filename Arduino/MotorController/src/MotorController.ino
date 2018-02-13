
#include <PID_v1.h>
#include <Encoder.h> 
#include "Motors.h"


//Motor(MotorPWM,MotorIn1,MotorIn2,EncoderA,EncoderB)
Motor M[4] = 
{ 
  {10,27,26,20,52},
  {11,29,28,21,53},
  {8,23,22,19,51},
  {9,25,24,18,50}
}; 

Motor FlagWheel(5,40,41,2,3);
//Motor Actuator(6,42,43,-1,-1);
              
unsigned long lastMilli = 0;
unsigned char incomingByte[2];
int k=0;

void setup() {
  Serial.begin(9600);
  incomingByte[0]=0;
  incomingByte[1]=0;
  for(int j=0;j<4;j++) {
    M[j].run(STOP);
  }
}

void loop() {

  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
    
    if (Serial.available()>0) {   
      incomingByte[0]=Serial.read();
      incomingByte[1]=Serial.read(); 
    } 
   
    int cmd = incomingByte[0];
    int Speed = incomingByte[1];
    if (cmd==0) {                   //STOP
      for(int j=0;j<4;j++) {
        M[j].run(STOP);
        M[j].Setpoint = 0;      
      }
    }
    else if (cmd==1) {             //FORWARD
      for(int j=0;j<4;j++) {
        M[j].run(FORWARD);
        M[j].Setpoint = Speed;
      }
      Serial.println(cmd);
    }
    else if (cmd==2) {              //BACKWARD
      for(int j=0;j<4;j++) {
        M[j].run(BACKWARD);
        M[j].Setpoint = Speed;
      }
    }      
    else if (cmd==3) {              //Strafe Right
      M[0].run(BACKWARD);
      M[1].run(FORWARD);
      M[2].run(BACKWARD);
      M[3].run(FORWARD);
      for(int j=0;j<4;j++) {
        M[j].Setpoint = Speed;
      }   
    }       
    else if (cmd==4) {              //Strafe Left
      M[0].run(FORWARD);
      M[1].run(BACKWARD);
      M[2].run(FORWARD);
      M[3].run(BACKWARD);
      for(int j=0;j<4;j++) {
        M[j].Setpoint = Speed;
      }   
    }  
    else if (cmd==5) {                //Turn Right
      M[0].run(BACKWARD);
      M[1].run(FORWARD);
      M[2].run(FORWARD);
      M[3].run(BACKWARD);
      for(int j=0;j<4;j++) {
        M[j].Setpoint = Speed;
      }   
    }  
    else if (cmd==6) {                //Turn Left      
      M[0].run(FORWARD);
      M[1].run(BACKWARD);
      M[2].run(BACKWARD);
      M[3].run(FORWARD);
      for(int j=0;j<4;j++) {
        M[j].Setpoint = Speed;
      }   
    } 
    else if(cmd == 7){                //Diagonal(Forward-Right)      
      M[0].Setpoint = 0;              //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = 0;
      M[3].Setpoint = Speed;
      
        
      M[0].run(STOP);                //Give direction of each motor
      M[1].run(FORWARD);
      M[2].run(STOP);
      M[3].run(FORWARD);
    }
    else if(cmd == 8){                //Diagonal(Forward-Left)      
      M[0].Setpoint = Speed;          //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = 0;
      
      M[0].run(FORWARD);
      M[1].run(STOP);
      M[2].run(FORWARD);
      M[3].run(STOP);
    }
    else if(cmd == 9){                //Diagonal(Back-Right)      
      M[0].Setpoint = Speed;              //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = 0;
      
      M[0].run(BACKWARD);
      M[1].run(STOP);
      M[2].run(BACKWARD);
      M[3].run(STOP);
    }
    else if(cmd == 10){                //Diagonal(Back-Left)      
      M[0].Setpoint = 0;              //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = 0;
      M[3].Setpoint = Speed;    
    
      M[0].run(STOP);
      M[1].run(BACKWARD);
      M[2].run(STOP);
      M[3].run(BACKWARD);
    }
    else if(cmd == 11){                //Cornering(BRC)      
      M[0].Setpoint = 0;              //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = 0;
    
      M[0].run(STOP);
      M[1].run(FORWARD);
      M[2].run(FORWARD);
      M[3].run(STOP);
    }
    else if(cmd == 12){                //Cornering(BLC)     
      M[0].Setpoint = Speed;           //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = 0;
      M[3].Setpoint = Speed;
   
      M[0].run(FORWARD);
      M[1].run(STOP);
      M[2].run(STOP);
      M[3].run(FORWARD);
    }
    else if(cmd == 13){                //Cornering(FRC)    
      M[0].Setpoint = 0;               //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = 0;
    
      M[0].run(STOP);
      M[1].run(BACKWARD);
      M[2].run(BACKWARD);
      M[3].run(STOP);
    }
    else if(cmd == 14){                //Cornering(FLC)     
      M[0].Setpoint = Speed;           //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = 0;
      M[3].Setpoint = Speed;
      
      M[0].run(BACKWARD);
      M[1].run(STOP);
      M[2].run(STOP);
      M[3].run(BACKWARD);
    }
    else if(cmd == 15){                //Cornering(Rear-Axis Right)     
      M[0].Setpoint = Speed;           //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = 0;
      M[3].Setpoint = 0;
    
      M[0].run(BACKWARD);
      M[1].run(FORWARD);
      M[2].run(STOP);
      M[3].run(STOP);
    }
    else if(cmd == 16){                //Cornering(Rear-Axis Left)     
      M[0].Setpoint = Speed;           //Set the speed of each motor
      M[1].Setpoint = Speed;          
      M[2].Setpoint = 0;
      M[3].Setpoint = 0;
    
      M[0].run(FORWARD);
      M[1].run(BACKWARD);
      M[2].run(STOP);
      M[3].run(STOP);
    }
    else if(cmd == 17){               //Cornering(Front-Axis Right)     
      M[0].Setpoint = 0;              //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = Speed;
 
      M[0].run(STOP);
      M[1].run(STOP);
      M[2].run(FORWARD);
      M[3].run(BACKWARD);
    }
    else if(cmd == 18){               //Cornering(Front-Axis Left)     
      M[0].Setpoint = 0;              //Set the speed of each motor
      M[1].Setpoint = 0;          
      M[2].Setpoint = Speed;
      M[3].Setpoint = Speed;
    
      M[0].run(STOP);
      M[1].run(STOP);
      M[2].run(BACKWARD);
      M[3].run(FORWARD);
    }
    else if(cmd == 19){               //Arc (Right / Left Backwards)     
      M[1].Setpoint += Speed;         //Increase speed of right motors 
      M[2].Setpoint += Speed;
    }
    else if(cmd == 20){               //Arc (Left / Right Backwards)     
      M[0].Setpoint += Speed;         //Increase speed of right motors 
      M[4].Setpoint += Speed;
    }
    else if(cmd == 21){               //Spin Wheel CW     
      long start = FlagWheel.getPosition();
      FlagWheel.setSpeed(45);
      FlagWheel.run(FORWARD);
      while (FlagWheel.getPosition() < start+9600) { 
      }
      FlagWheel.run(STOP);
    }
    else if(cmd == 22){               //Spin Wheel CCW     
      long start = FlagWheel.getPosition();
      FlagWheel.setSpeed(45);
      FlagWheel.run(BACKWARD);
      while (FlagWheel.getPosition() < start+9600) { 
      }
      FlagWheel.run(STOP);
    }
/*
    else if(cmd == 23){               //Linear Actuator Up     
      Serial.print("UP");
      Actuator.setSpeed(255);
      Actuator.run(FORWARD);
    }
    else if(cmd == 24){               //Linear Actuator Down     
      Serial.print("DOWN");
      Actuator.setSpeed(220);
      Actuator.run(BACKWARD);
    }
    else if(cmd == 25){               //Linear Actuator Stop     
      Serial.print("STOP");
      Actuator.run(STOP);
    }
*/
    else {                             //STOP
      for(int j=0;j<4;j++){
        M[j].run(STOP);
        M[j].Setpoint = 0;      
      }
    }    

 
   
    for(int j=0;j<4;j++) {
      M[j].updatePID();
    }  


  k++;
  Serial.flush();
  } 
}
