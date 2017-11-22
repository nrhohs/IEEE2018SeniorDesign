#include <stdio.h>
#include <wireingPi.h>

int main(int argc,char **argv){
wiringPiSetup();
pinMode(0,OUTPUT);
for(;;){
 digitalWrite (0, HIGH); delay(500);
 digitalWrtie (0, LOW); delay(500);
}
return 0;
}
