#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_VL6180X.h"
#include <stdlib.h>
#include "wiringPi.h"
#include <stdint.h>

#define TCAADDR 0x70

int main(){
  Adafruit_VL6180X vl = Adafruit_VL6180X();  
  int fd = vl.begin(1); 
  char buf[15]
  int mux;
  snprintf(buf,15,"/dev/i2c-1");
  mux=open(buf,O_RDWR)
  printf("Adafruit VL6180x test!\n");
  if (fd<0){
    printf("Failed to find sensor\n");
    while (1);
  }
  printf("Sensor found!\n");


while(1) {
//  float lux = vl.readLux(VL6180X_ALS_GAIN_5);

  
   uint8_t range = vl.readRange(fd);
   uint8_t status = vl.readRangeStatus(fd);

  if (status == VL6180X_ERROR_NONE) {
    printf("Range: "); printf("%d\t\r",range);
  }

  // Some error occurred, print it out!
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) {
    printf("System error\r");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) {
    printf("ECE failure\r");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) {
    printf("No convergence\r");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) {
    printf("Ignoring range\r");
  }
  else if (status == VL6180X_ERROR_SNR) {
    printf("Signal/Noise error\r");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) {
    printf("Raw reading underflow\r");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) {
    printf("Raw reading overflow\r");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) {
    printf("Range reading underflow\r");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) {
    printf("Range reading overflow\r");
  }
  delay(50);
  fflush(stdout);
  printf("                                 \r");
}
}

void tcaselect(uint8_t i) {
{
  if (i>7)
    return;
  char data_write[3];
  data_write[0] = (TCAADDR>>8) & 0xFF;
  data_write[1] = (TCAADDR) & 0xFF;
  data_write[2] = (1 << i);
  write(fd, data_write, 3);

#if defined(I2C_DEBUG)
  printf("\t$"); printf("%X",address); printf(" = 0x"); printf("%X\n",data);
#endif
}
