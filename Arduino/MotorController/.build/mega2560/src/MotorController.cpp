#include <Arduino.h>
#include <PID_v1.h>
#include <Encoder.h> 
#include "Motors.h"
void setup();
void loop();
#line 1 "src/MotorController.ino"

//#include <PID_v1.h>
//#include <Encoder.h> 
//#include "Motors.h"


//Motor(MotorPWM,MotorIn1,MotorIn2,EncoderA,EncoderB)
Motor M[4] = 
{ 
  {10,27,26,20,52},
  {11,29,28,21,53},
  {8,23,22,19,51},
  {9,25,24,18,50}
}; 

Motor FlagWheel(5,40,41,2,3);

              
int ActuatorPins[3] = {6,38,39};

unsigned long lastMilli = 0;
unsigned char incomingByte[2];
int k=0;

void setup() {
  Serial.begin(9600);
  analogReference(INTERNAL2V56);
  FlagWheel.changeTunings(0.06,0.01,0.03);
  int current = analogRead(A0);
  incomingByte[0]=0;
  incomingByte[1]=0;
  for(int j=0;j<4;j++) {
    M[j].run(STOP);
  }
  for(int j=0;j<3;j++) {
    pinMode(ActuatorPins[j],OUTPUT);
  }
}

void loop() {

    
    if (Serial.available()>0) {   
      incomingByte[0]=Serial.read();
      delay(10);
      incomingByte[1]=Serial.read(); 
      delay(10);
    } 
   
    int cmd = incomingByte[0];
    int Speed = incomingByte[1];
    //int cmd = 22;
    //int Speed = 0;
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
      for(int j=1;j<4;j+=2) {
        M[j].Setpoint = Speed;
      }   
      for(int j=0;j<4;j+=2) {
        M[j].Setpoint = Speed;
      }   
    }       
    else if (cmd==4) {              //Strafe Left
      M[0].run(FORWARD);
      M[1].run(BACKWARD);
      M[2].run(FORWARD);
      M[3].run(BACKWARD);
      for(int j=1;j<4;j+=2) {
        M[j].Setpoint = Speed;
      }   
      for(int j=0;j<4;j+=2) {
        M[j].Setpoint = Speed;
      }   
/*
      for(int j=0;j<4;j++) {
        M[j].setSpeed(Speed);
      }   
*/
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
      M[3].Setpoint += Speed;
    }
    else if(cmd == 20){               //Arc (Left / Right Backwards)     
      M[1].Setpoint = Speed;         //Increase speed of right motors 
      M[3].Setpoint = Speed;
    }
    else if(cmd == 21){               //Spin Wheel CW     
      long start = FlagWheel.getPosition();
      FlagWheel.setSpeed(40);
      FlagWheel.run(FORWARD);
      while (FlagWheel.getPosition() > start-9600) { 
      }
      FlagWheel.run(STOP);
    }
    else if(cmd == 22){               //Spin Wheel CCW     
      long start = FlagWheel.getPosition();
      FlagWheel.Setpoint=30;
      //FlagWheel.setSpeed(40);
      FlagWheel.run(BACKWARD);
      int i =0;
      int avg = 0;
      while (FlagWheel.getPosition() < start+9600) { 
/*
	Serial.print("Position: ");
	Serial.print(FlagWheel.getPosition());
	Serial.print("  Current: ");
	int current = analogRead(A0);
	Serial.println(current);
	if (i==0)
	  avg-=current;
	if (current != 0) {
	  i++;
	  avg+=current;
	}
*/
        FlagWheel.updatePID();
      }
	Serial.print("\n AVERAGE: ");
	Serial.println(avg/i);
      FlagWheel.Setpoint=0;
      FlagWheel.run(STOP);
	delay(10000);
    }
    else if(cmd == 23){               //Linear Actuator Down     
      digitalWrite(ActuatorPins[2],LOW);
      digitalWrite(ActuatorPins[1],HIGH);
      analogWrite(ActuatorPins[0], 220);
    }
    else if(cmd == 24){               //Linear Actuator Up     
      digitalWrite(ActuatorPins[1],LOW);
      digitalWrite(ActuatorPins[2],HIGH);
      analogWrite(ActuatorPins[0], 255);
    }
    else if(cmd == 25){               //Linear Actuator Stop     
      digitalWrite(ActuatorPins[1],LOW);
      digitalWrite(ActuatorPins[2],LOW);
      analogWrite(ActuatorPins[0], 0);
    }
    else {                             //STOP
      for(int j=0;j<4;j++){
        M[j].run(STOP);
        M[j].Setpoint = 0;      
      }
    }    

 
  if((millis()-lastMilli) >= LOOPTIME) {
    lastMilli = millis();
    for(int j=0;j<4;j++) {
      M[j].updatePID();
    }  

    k++;
  } 

  Serial.flush();
}
