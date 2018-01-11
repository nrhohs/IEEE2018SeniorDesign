#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_VL6180X.h"
#include <stdlib.h>
#include <stdint.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>

int main(){
  Adafruit_VL6180X vl1 = Adafruit_VL6180X();  
  Adafruit_VL6180X vl2 = Adafruit_VL6180X();  
  VL53L0X_Error VL53L0XStatus = VL53L0X_ERROR_NONE;
  VL53L0X_Dev_t MyDevice;
  VL53L0X_Dev_t *pMyDevice = &MyDevice;
  VL53L0X_RangingMeasurementData_t = RangingMeasurementData;



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
  if (vl2.begin()>0) 
    printf("No Sensor Found\n");

  data_write[0] = 1<<0;
  write(mux, data_write, 1);
  pMyDevice->I2cDevAddr = 0x29; 
  pMyDevice->fd = VL53L0X_i2c_init("/dev/i2c-1");

while(1) {

  uint8_t range = vl1.readRange();
  uint8_t status = vl1.readRangeStatus();

  data_write[0] = 1<<2;
  write(mux, data_write, 1);
  
   

  if (status == VL6180X_ERROR_NONE) {
    printf("TOF1 Range: "); printf("%3d   ",range);
  }
  else {
    printf("TOF1 Range: "); printf("XXX   ");
  }

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

  VL53L0XStatus = VL53L0X_GetRangingMeasurementData(pMyDevice,&RangingMeasurementData);

  if (RangingMeasurementData.RangeStatus == 0) {
    printf("TOF2 Range: "); printf("%4d   ",RangingMeasurementData.RangeMilliMeter);
  }
  else {
    printf("TOF2 Range: "); printf("XXXX   ");
  }


  fflush(stdout);
  usleep(50000);
  printf("                                 \r");
}
}
