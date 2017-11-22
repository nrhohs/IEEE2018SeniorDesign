#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <wiringPi.h>

void IROutput(int);
void LogicalZero(); 
void LogicalOne(); 

int main(void){
  srand(time(NULL));
  wiringPiSetup();
  pinMode(0,OUTPUT);
  int random = rand()%7;
  for(;;){
    IROutput(random);
  }
  return 0;
}


void IROutput(int x){
  digitalWrite(0,HIGH);
  delay(9);
  digitalWrite(0,LOW);
  delayMicroseconds(4500);
  int i;
  for(i=0;i<5;i++){
    LogicalZero();
  }
  if((x&0x4) != 0)
    LogicalOne();
  else
    LogicalZero();
  if((x&0x2) != 0)
    LogicalOne();
  else
    LogicalZero();
  if((x&0x1) != 0)
    LogicalOne();
  else
    LogicalZero();
  digitalWrite(0,HIGH);
  delayMicroseconds(560);
  digitalWrite(0,LOW);
  delay(1000);
  return;
}     




void LogicalZero(){ 
  digitalWrite(0,HIGH);
  delayMicroseconds(560);
  digitalWrite(0,LOW);
  delayMicroseconds(560);
  return;
}
void LogicalOne(){ 
  digitalWrite(0,HIGH);
  delayMicroseconds(560);
  digitalWrite(0,LOW);
  delayMicroseconds(1680);
  return;
}
