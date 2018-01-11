////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


#include "RTIMULib.h"
#include "RTMath.h"
#include <stdbool.h>
#include <stdio.h>
#include "Adafruit_VL6180x.h"
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "rs232.h"
#include <wiringPi.h>
#include <cmp23017.h>
#include <lcd.h>
#include <softPwm.h>

#define BUF_SIZE 8

RTIMU *imuInit();
void waitOnTOF(vl);
double getCurrImu(RTIMU *);
void waitOnImu(RTIMU *,double);


int main()
{
    Adafruit_VL6180X vl = Adafruit_VL6180X();
    int fd = vl.begin(3);
       printf("Adafruit VL6180x test!\n");
       if (fd<0){
         printf("Failed to find sensor\n");
       while (1);
       }
       printf("Sensor found!\n");


    RTIMU *imu = imuInit();

    double startYaw=getCurrImu(imu);


    //Robot test begin
    int display, j;
    wiringPiSetup();
    mcp23017Setup (100, 0x20);
    for (j=0;j<16;j++)
	pinMode(100+j,OUTPUT);
    digitalWrite(101,0);
    display=lcdinit(2,16,4,100,102,103,104,105,106,0,0,0,0);
    lcdHome(display);
    lcdClear(display);
    lcdPosition(display,0,0);
    pinMode(0, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    softPwmCreate(3, 50, 100);
    softPwmCreate(2, 50, 100);
    softPwmCreate(0, 50, 100);
    softPwmWrite(3,100);
    softPwmWrite(2,0);
    softPwmWrite(0,0);

    int cport_nr=24;
    int bdrate=9600;
    int i=0;
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit

  unsigned char str_send[BUF_SIZE];
  str_send[0] = 0;
  str_send[1] = 30;
  
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */
  
  while(1)
  {

  /*
  if(i<3) {
    str_send[0] = 1;
  }
  else if(i<4) {
    str_send[0] = 0;
  }
  else if(i<7) {
    str_send[0] = 3;
  }
  else if(i<8) {
    str_send[0] = 0;
  }
  else if(i<11) {
    str_send[0] = 2;
  }
  else if(i<12) {
    str_send[0] = 0;
  }
  else if(i<16) {
    str_send[0] = 4;
  }
  else if(i<17) {
    str_send[0] = 0;
  }
  else {
    i=-1;
  }
  */


    //Go fwd
    usleep(1000000);  /* sleep for 1 Second */
    str_send[0]=1;
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
        lcdPosition(display,0,0);
        lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    waitOnTOF(vl);
    
    //stop
    str_send[0]=0;
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
        lcdPosition(display,0,0);
        lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    double currYaw=getCurrImu(imu);
    usleep(1000000);  /* sleep for 1 Second */


    //Turn Right
    str_send[0]=5;
    str_send[0]=0;
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
        lcdPosition(display,0,0);
        lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    waitOnImu(imu,currYaw);
    
    //stop
    str_send[0]=0;
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
	printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
        lcdPosition(display,0,0);
        lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    double currYaw=getCurrImu(imu);
    usleep(1000000);  /* sleep for 1 Second */
  }
  return(0);

}

void waitOnTOF(Adafruit_VL6180X *vl,int fd) {
    
    while (1) {
            uint8_t range = vl.readRange(fd);
 	    uint8_t status = vl.readRangeStatus(fd);

	    if (range <= 50)
		break;
    }
}



RTIMU *imuInit() {
    //  Using RTIMULib here allows it to use the .ini file generated by RTIMULibDemo.
    //  Or, you can create the .ini in some other directory by using:
    //      RTIMUSettings *settings = new RTIMUSettings("<directory path>", "RTIMULib");
    //  where <directory path> is the path to where the .ini file is to be loaded/saved


    RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

    RTIMU *imu = RTIMU::createIMU(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }

    //  This is an opportunity to manually override any settings before the call IMUInit

    //  set up IMU

    imu->IMUInit();

    //  this is a convenient place to change fusion parameters

    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(false);

}

double getCurrImu(RTIMU *imu) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTVector3 vec=imuData.fusionPose;
    double yaw=vec.z() * RTMATH_RAD_TO_DEGREE; 
    return yaw;
}


void waitOnImu(RTIMU *imu, double start) {
    double endAt = (start + 90.0)
    endAt = fmod(endAt,360.0);
    while (1) {
    	if (getCurrImu(imu) >= endAt)
	    break;
    }
}
    
    

