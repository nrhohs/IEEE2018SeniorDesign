/******************************************************
Robot.cpp
This file initializes all robot sensors, displays, and
connections, and then calls functions from the navigation
library to navigate the course

Cmdline args:
"-sensor" does not move the robot but gives a readout of all sensors

********************************************************/
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "Libraries/rs232.h"
#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>
#include <softPwm.h>
#include "Libraries/navigation.h"
#include "Libraries/SensorLib2.h"
#include <string.h>


int main(int argc, char* argv[])
{

    //Initializes MUX
    MUX *mux = initMUX();

    //Initializes TOFs
    //Let's create an array of TOF's so that we can pass them to a function	
    TOF *tof0 = newTOF(0, mux, 0);	//front facing SR
    TOF *tof1 = newTOF(0, mux, 1);	//right facing SR (front)
    TOF *tof2 = newTOF(0, mux, 2);	//right facing SR (back)
    TOF *tof3 = newTOF(0, mux, 3);	//back facing SR
    TOF *tof4 = newTOF(0, mux, 4);	//left facing SR  (back)
    TOF *tof5 = newTOF(1, mux, 5);	//left facing SR  (front)
    TOF *tof6 = newTOF(1, mux, 6);	//right facing LR (box TOF)
//    TOF *tof7 = newTOF(1, mux, 7);	//front facing LR (dropoff TOF)

    //Initializes IMU
//	RTIMU *imu = imuInit();
    //We can make a robot object that takes in the TOF* array RTIMU* and MUX*
    //The code directly below would be moved to an InitRobot() function
    //If initialization occurs correctly a state machine can begin with the Robot
    //object as the argument to the function	

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
 
//	int rawcode = routeread();
        int rawcode = 7;
        printf("%s\n",argv[1]);
	//provides sensor readouts
	if (strcmp(argv[1],"-sensor")==0){
		while(1){
			//senor readout code
			printf("TOF0: %d  ",getDistance(tof0));
			printf("TOF1: %d  ",getDistance(tof1));
			printf("TOF2: %d  ",getDistance(tof2));
			printf("TOF3: %d  ",getDistance(tof3));
			printf("TOF4: %d  ",getDistance(tof4));
			printf("TOF5: %d  ",getDistance(tof5));
			printf("TOF6: %d\n",getDistance(tof6));
		}
	}

	while(1){
		/* TASK 1 */
		if (rawcode < 4){
			fwd_waitOnTOF(20, tof5, display, 20);
			stop(1000000,display);
			bwd_waitOnTOF(20, tof5, display, 485);
		}
		if (rawcode >= 4){
			bwd_waitOnTOF(20, tof5, display, 660);
			stop(1000000,display);
			fwd_waitOnTOF(20, tof5, display, 485);
		}

		/* TRANSIT 1-->2 */
		//Rotate 90 to go down ramp
		turnRight_waitOnIMU(5,imu,90.0,display);
		//Go down ramp
		fwd_timed(20,2000000,display);
		//Find box
		fwd_waitOnTOF(10,tof5,559,display);
		//Rotate 90 to orient flag mech
		turnRight_waitOnIMU(5,imu,90.0,display);

		/* TASK 2 */
		if (rawcode%4 < 2){
		//B button 0
			bwd_timed(20,4000000,display);
			stop(1000000,display);
		}
		else{
		//B button 1
			fwd_timed(20,4000000,display);
			stop(1000000,display);
		}
		//for future purposes
		  
	}

	return(0);
}

