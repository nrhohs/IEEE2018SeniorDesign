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
#include "Libraries/Adafruit_VL6180x.h"
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "Libraries/rs232.h"
#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>
#include <softPwm.h>

#define BUF_SIZE 8

RTIMU *imuInit();
void waitOnTOF(Adafruit_VL6180X *);
double getCurrImu(RTIMU *);
void waitOnImu(RTIMU *,double,Adafruit_VL6180X *);


int main()
{
    //Checks for connection to TOF sensor
    Adafruit_VL6180X vl = Adafruit_VL6180X();
    int fd = vl.begin();
       printf("Adafruit VL6180x test!\n");
       if (fd<0){
         printf("Failed to find sensor\n");
       while (1);
       }
       printf("Sensor found!\n");

    //Initializes IMU
    RTIMU *imu = imuInit();



    //Robot Initialization & Setup
    int display, j;
    wiringPiSetup();
    mcp23017Setup (100, 0x20);
    for (j=0;j<16;j++)
	pinMode(100+j,OUTPUT);
    digitalWrite(101,0);
    display=lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);
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
  char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit

  //Initializing motor commands
  unsigned char str_send[BUF_SIZE];
  str_send[0] = 0;							//init direction to STOP
  str_send[1] = 30;							//init speed to 30 out of 255
  
  //Checks for serial connection
  if(RS232_OpenComport(cport_nr, bdrate, mode))
  {
    printf("Can not open comport\n");
    return(0);
  }

  usleep(2000000);  /* waits 2000ms for stable condition */
  
  /*****************
  Actual Robot Start
  *****************/
  while(1)
  {

    //Go fwd
    usleep(1000000);  /* sleep for 1 Second */
    str_send[0]=1;                                                      //Set direction to FWD
    RS232_SendByte(cport_nr,str_send[0]);                               //Send direction
    RS232_SendByte(cport_nr,str_send[1]);				//Send speed
    printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);    //cmdline output of command
    lcdPosition(display,0,0);						//Init position of lcd to beginning
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);	//Output command to lcd
    waitOnTOF(&vl);							//Call TOF function for wait
    
    //stop
    str_send[0]=0;							//Set direction to STOP
    RS232_SendByte(cport_nr,str_send[0]);				//Send direction
    RS232_SendByte(cport_nr,str_send[1]);				//Send speed
    printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);	//cmdline output of command
    lcdPosition(display,0,0);						//Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);	//Output command to lcd
    double currYaw=getCurrImu(imu);					//checks yaw on IMU
    usleep(1000000);  /* sleep for 1 Second */


    //Turn Right
    str_send[0]=5;							//Set direction to TURN RIGHT
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    waitOnImu(imu,currYaw,&vl);
    //waitOnImu calls to wait for an IMU change using previously checked yaw value
    
    //stop
    str_send[0]=0;
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: '%3d %3d'\n", str_send[0],str_send[1]);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    currYaw=getCurrImu(imu);
    usleep(1000000);  /* sleep for 1 Second */
  }


  return(0);
}

void waitOnTOF(Adafruit_VL6180X *vl) {
    uint8_t throwaway = vl->readRange();
    delay(100);
    uint8_t range = vl->readRange();
    delay(100);
    while (1) {
	    printf("%d\n",range);
	    if (range <= 100)
		break;
            delay(100);
            range = vl->readRange();
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
    return imu;
}

double getCurrImu(RTIMU *imu) {
    RTIMU_DATA imuData = imu->getIMUData();
    RTVector3 vec=imuData.fusionPose;
    double yaw=vec.z() * RTMATH_RAD_TO_DEGREE; 
    printf("current:%f  ",yaw);
    return yaw;
}


void waitOnImu(RTIMU *imu, double start, Adafruit_VL6180X *vl) {
    static double beginValue = getCurrImu(imu);
    static int i = 0; 
    double endAt = 0;
    if((i % 4) == 0){

        endAt = beginValue;
    }    
    else if((i % 4) == 1){

        endAt = beginValue + 90.0;
    }
    else if((i % 4) == 2){

        endAt = beginValue + 180.0;
    }
    else if((i % 4) == 3){

        endAt = beginValue + 270.0;
    }
    endAt = fmod(endAt,360.0);
    double currYaw= 0.0f;  
    usleep(imu->IMUGetPollInterval()*1000);
    while (1) { 
    while (imu->IMURead()) {
	currYaw=getCurrImu(imu);
        printf("target:  %f\n",endAt);
    	if (currYaw >= endAt  && currYaw <= endAt+10)
	    break;
        delay(100);
    }
    	if (currYaw >= endAt  && currYaw <= endAt+10)
	    break;
    }
    i = i +1;
}
    
    

