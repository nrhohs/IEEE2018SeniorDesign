#include <stdio.h>
#include "wiringPi.h"
#include <wiringPiI2C.h>
#include <stdlib.h>

int main(){
if(wiringPiI2CSetup(29)==-1);
  return -1;
return 0;
}
