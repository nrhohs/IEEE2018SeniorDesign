/********************************************************
					Robot.cpp
This file initializes all robot sensors, displays, and
connections, and then calls functions from the navigation
library so that the 
********************************************************/
#include "RTIMULib.h"
#include "RTMath.h"
#include <stdbool.h>
#include <stdio.h>
#include "Libraries/inc/Adafruit_VL6180x.h"
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "Libraries/rs232.h"
#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>
#include <softPwm.h>
#include "navigation.h"

int main()
{

    //Initializes MUX
    MUX *mux = initMUX();

    //Initializes TOFs
    TOF *tof0 = newTOF(0, mux, 0);	//front facing SR
    TOF *tof1 = newTOF(0, mux, 1);	//right facing SR (front)
    TOF *tof2 = newTOF(0, mux, 2);	//right facing SR (back)
    TOF *tof3 = newTOF(0, mux, 3);	//back facing SR
    TOF *tof4 = newTOF(0, mux, 4);	//left facing SR  (back)
    TOF *tof5 = newTOF(0, mux, 5);	//left facing SR  (front)
    TOF *tof6 = newTOF(1, mux, 6);	//right facing LR (box TOF)
    TOF *tof7 = newTOF(1, mux, 7);	//front facing LR (dropoff TOF)

    //Initializes IMU
	IMU *imu = imuInit();



    //Display Initialization
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

	//Initializing serial connection
	int bdrate = 9600;
	char mode[]={'8','N','1',0}; // 8 data bits, no parity, 1 stop bit
	int cport_nr = 24;
	if(RS232_OpenComport(cport_nr, bdrate, mode)){
		printf("Can not open comport\n");
		return(0);
	}

	usleep(2000000);  /* waits 2000ms for stable condition */

	/*****************
	Actual Robot Start
	*****************/
	while(1){
		//Test nav to just make it move in a box
		fwd_waitOnTOF(30, *tof0, display);
		stop(1000000, display);
		turnRight_waitOnIMU(30, *imu, 90, display);
		stop(1000000, display);
	}

	return(0);
}
