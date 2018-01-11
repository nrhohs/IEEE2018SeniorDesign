#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_VL6180X.h"
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>

int main(){
  Adafruit_VL6180X vl = Adafruit_VL6180X();  
  char buf[15];
  snprintf(buf,15,"/dev/i2c-1");
  int mux = open(buf,O_RDWR);
  if(ioctl(mux,I2C_SLAVE,0x70)<0)
    printf("error\n");
  char data_write[1];
  data_write[0] = 1<<2;
  write(mux, data_write, 1);
  int fd = vl.begin(1); 
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
  fflush(stdout);
  usleep(50000);
  printf("                                 \r");
}
}
