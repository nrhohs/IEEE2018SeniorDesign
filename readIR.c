#include <stdio.h>
#include <stdlib.h>
#include <time.h>
//#include <wiringPi.h>

void IROutput(int);
void LogicalZero(); 
void LogicalOne(); 

int main(void){
  srand(time(NULL));
//wiringPiSetup();
//inMode(0,OUTPUT);
  int i;
  for(i=0;i<8;i++){
    IROutput((rand()%7));
  }
  return 0;
}


void IROutput(int x){
// digitalWrite(0,HIGH);
// delay(9);
// digitalWrite(0,LOW);
// delayMiocroseconds(4500);
  printf("%d :",x); 
  int i;
  for(i=0;i<5;i++){
    printf("0");
  }
  if((x&0x4) != 0)
    printf("1");
  else
    printf("0");
  if((x&0x2) != 0)
    printf("1");
  else
    printf("0");
  if((x&0x1) != 0)
    printf("1");
  else
    printf("0");
  printf("\n");
  return;
}     




void LogicalZero(){ 
//  digitalWrite(0,HIGH);
//  delayMicroseconds(560);
// digitalWrite(0,LOW);
// delayMicroseconds(560);
// return;
}
void LogicalOne(){ 
// digitalWrite(0,HIGH);
// delayMicroseconds(560);
// digitalWrite(0,LOW);
// delayMicroseconds(1680);
// return;
}
