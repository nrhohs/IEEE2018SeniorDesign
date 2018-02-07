#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_VL6180X.h"
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include "VL53L0X.h"
#include <sys/ioctl.h>



 
int main(){
  Adafruit_VL6180X vl1 = Adafruit_VL6180X();  
//  Adafruit_VL6180X vl2 = Adafruit_VL6180X();  
  
  VL53L0X TOF;
  TOF.init();
  TOF.setTimeout(200);

  char buf[15];
  snprintf(buf,15,"/dev/i2c-1");
  int mux = open(buf,O_RDWR);
  if(ioctl(mux,I2C_SLAVE,0x70)<0)
    printf("error\n");
  char data_write[1];
  data_write[0] = 1<<2;
  write(mux, data_write, 1);
  vl1.begin();

  data_write[0] = 1<<7;
  write(mux, data_write, 1);
/*
  if (vl2.begin()>0) 
    printf("No Sensor Found\n");

    data_write[0] = 1<<0;
    write(mux, data_write, 1);
*/

while(1) {
  data_write[0] = 1<<2;
  write(mux, data_write, 1);
  
  uint8_t range = vl1.readRange();
  uint8_t status = vl1.readRangeStatus();
   

  if (status == VL6180X_ERROR_NONE) {
    printf("TOF1 Range: "); printf("%3d   ",range);
  }
  else {
    printf("TOF1 Range: "); printf("XXX   ");
  }
/*
  data_write[0] = 1<<7;
  write(mux, data_write, 1);
  
   range = vl2.readRange();
   status = vl2.readRangeStatus();

  if (status == VL6180X_ERROR_NONE) {
    printf("TOF2 Range: "); printf("%3d   ",range);
  }
  else {
    printf("TOF2 Range: "); printf("XXX   ");
  }
*/

  data_write[0] = 1<<0;
  write(mux, data_write, 1);

  uint16_t distance = TOF.readRangeSingleMillimeters();
    if (TOF.timeoutOccurred()) {
      printf("TOF3 Range: "); printf("XXXX  \r");
    }
    else {
      printf("TOF3 Range: "); printf("%4d   \r",distance);
    }

  fflush(stdout);
  usleep(50000);
  printf("                                 \r");
}
}
