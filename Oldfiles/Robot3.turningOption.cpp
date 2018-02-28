/******************************************************
Robot3.cpp
This file initializes all robot sensors, displays, and
connections, and then calls functions from the navigation
library to navigate the course

Cmdline args:
"-sensor" does not move the robot but gives a readout of all sensors
"-route" follwed by a space and a number hardcodes an IR route
    * Valid routes are 1 - 8.

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
#include <pigpio.h>

void correctAngle(RTIMU *,int,double);

int main(int argc, char* argv[])
{
    //Stop robot on Exit
    std::atexit(stopExit);

    //    printf("%d, %s, %s", argc, argv[0], argv[1]);
    //Initialize Stop Switch
    if(gpioInitialise()<0) return 1;
    gpioSetMode(21, PI_INPUT);
    gpioSetPullUpDown(21, PI_PUD_UP);
//    gpioSetISRFunc(21, FALLING_EDGE, -1, STOPISR);

    //Initializes MUX
    MUX *mux = initMUX();

    //Initializes TOFs
    //Let's create an array of TOF's so that we can pass them to a function	
    TOF *tof0 = newTOF(0, mux, 0);	//front facing SR
    TOF *tof1 = newTOF(0, mux, 1);	//right facing SR (front)
    TOF *tof2 = newTOF(1, mux, 2);	//right facing SR (back)
    TOF *tof3 = newTOF(0, mux, 3);	//back facing SR
    TOF *tof4 = newTOF(0, mux, 4);	//left facing SR  (back)
    TOF *tof5 = newTOF(1, mux, 5);	//left facing SR  (front)
    TOF *tof6 = newTOF(1, mux, 6);	//right facing LR (box TOF)
//    TOF *tof7 = newTOF(1, mux, 7);	//front facing LR (dropoff TOF)

    //Initializes IMU
	RTIMU *imu = imuInit();
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

        int rawcode = 0;
	//int rawcode=7;
	//provides sensor readouts
	if (argc>1){
	   for (int i=0;i<argc;i++) {
		if (strcmp(argv[i],"-sensor")==0){
			while(1){
				//senor readout code
				printf("TOF0: %d  ",getDistance(tof0));
				printf("TOF1: %d  ",getDistance(tof1));
				printf("TOF2: %d  ",getDistance(tof2));
				printf("TOF3: %d  ",getDistance(tof3));
				printf("TOF4: %d  ",getDistance(tof4));
				printf("TOF5: %d  ",getDistance(tof5));
				printf("TOF6: %d  ",getDistance(tof6));
				printf("IMU:  %f\n",getCurrImuYaw(imu));
			}
		}
		else if (strcmp(argv[i],"-route")==0){
			rawcode=argv[i+1][0]-'0';
		}
		else if (strcmp(argv[i],"-pid")==0){
			while(1) {
            			fwd_waitOnTOF(10, tof5, display, 80);
	    			bwd_waitOnTOF(10,tof5,display,300);
			}
		}
	    }
	}
	else {
		rawcode = routeread(display);
	}

				printf("TOF0: %d  ",getDistance(tof0));
			//	printf("TOF1: %d  ",getDistance(tof1));
				printf("TOF2: %d  ",getDistance(tof2));
		//		printf("TOF3: %d  ",getDistance(tof3));
				printf("TOF4: %d  ",getDistance(tof4));
				printf("TOF5: %d  ",getDistance(tof5));
				printf("TOF6: %d  ",getDistance(tof6));
				printf("IMU:  %f\n",getCurrImuYaw(imu));

	stop(50000,display);


	if (rawcode < 4){
            printf("run A");
            fwd_waitOnTOF(100, tof5, display, 80);
	    stop(1000000,display);
	    bwd_waitOnTOF(100, tof5, display, 400 );
	}
	else{
	    printf("run B");
	    bwd_waitOnTOF(100,tof2,display,80);
	    stop(1000000,display);
	    fwd_waitOnTOF(100, tof2, display, 400);
	}


	//Rotate 90 to go down ramp
	//turnLeft_waitOnIMU(25,imu,90.0,display);
	turnLeft_waitOnIMU(75,imu,80.0,display,false);
	stop(500000,display);

	//Go down ramp
	fwd_timed(30,5000000,display);


	//Find box
	stop(1000000,display);
	fwd_waitOnTOF(75,tof5,display,500);
	stop(500000,display);


	//Rotate 90 to orient flag mech
	turnRight_waitOnIMU(75,imu,0.0,display,true);
        if(getCurrImuYaw(imu)<180)
	  turnLeft_waitOnIMU(30,imu,0.0,display,true);
        else
	  turnRight_waitOnIMU(30,imu,0.0,display,true);

	if (rawcode%4 < 2){
	//B button 0
	/************ Destination B *************************/
		bwd_waitOnTOF(50,tof2,display,160);
		stop(1000000,display);
		fwd_timed(40,800000,display);
		stop(1000000,display);
       		if(getCurrImuYaw(imu)<180)
	  		turnLeft_waitOnIMU(20,imu,0.0,display,true);
        	else
	  		turnRight_waitOnIMU(20,imu,0.0,display,true);

	/***************** Flag Wheel **************************/

	/*****************Option 1 - Strafing ******************/

		strafeLeft_waitOnTOF(150,tof4,display,100);
		stop(1000000,display);

       		if(getCurrImuYaw(imu)<180)
	  		turnLeft_waitOnIMU(20,imu,0.0,display,true);
        	else
	  		turnRight_waitOnIMU(20,imu,0.0,display,true);
		if(getDistance(tof2)>60)
			bwd_waitOnTOF(60,tof2,display,60);
		stop(100000,display);
//		strafeLeft_waitOnTOF(60,tof4,display,20);
		strafeLeft_timed(60,600000,display);
		stop(100000,display);

		fwd_waitOnTOF(5,tof0,display,100);
		stop(10000,display);
		strafeLeft_waitOnTOF(60,tof4,display,15);
		stop(10000,display);
		strafeLeft_timed(60,300000,display);
		stop(100000,display);

		//fwd_timed(10,350000,display);
		stop(500000,display);
		sendCommand(22,50,display);


	/*****************Option 2 - Turning *******************/
	/*
		turnRight_waitOnIMU(75,imu,90.0,display,true);
        	if(getCurrImuYaw(imu)>=90)
	  		turnLeft_waitOnIMU(30,imu,90.0,display,true);
        	else
			turnRight_waitOnIMU(30,imu,90.0,display,true);
		bwd_waitOnTOF(50,tof2,display,340);
		turnBackLeft_waitOnIMU(40,imu,0.0,display,true);
		stop(50000,display);
		fwd_waitOnTOF(30,tof2,display,300);
		stop(50000,display);
		strafeLeft_timed(100,750000,display);
		stop(50000,display);
		fwd_waitOnTOF(30,tof2,display,350);
		bwd_timed(20,350000,display);
		stop(50000,display);
		sendCommand(22,50,display);	// Spin wheel CW
	*/
	}
	else{
	//B button 1
	/****************** Destination B ***********************/
		fwd_waitOnTOF(75,tof5,display,150);
		stop(500000,display);
		bwd_timed(40,800000,display);
		stop(1000000,display);
       		if(getCurrImuYaw(imu)<180)
	  		turnLeft_waitOnIMU(40,imu,0.0,display,true);
        	else
	  		turnRight_waitOnIMU(40,imu,0.0,display,true);

	/***************** Flag Wheel **************************/

	/*****************Option 1 - Strafing ******************/
		strafeLeft_waitOnTOF(150,tof4,display,100);
		stop(1000000,display);
       		if(getCurrImuYaw(imu)<180)
	  		turnLeft_waitOnIMU(20,imu,0.0,display,true);
        	else
	  		turnRight_waitOnIMU(20,imu,0.0,display,true);
		if(getDistance(tof5)>100)
			fwd_waitOnTOF(60,tof5,display,60);
		stop(100000,display);
		//strafeLeft_timed(60,600000,display);
		strafeLeft_waitOnTOF(30,tof4,display,15);
		stop(100000,display);

		bwd_waitOnTOF(5,tof1,display,100);
		stop(100000,display);
		strafeLeft_waitOnTOF(60,tof4,display,15);
		stop(100000,display);
		strafeLeft_timed(60,250000,display);
		stop(100000,display);


//		bwd_timed(5,150000,display);
		stop(500000,display);
		sendCommand(22,50,display);

	/*****************Option 2 - Turning *******************/
	/*****************Option 2 - Turning *******************/
	/*
		turnLeft_waitOnIMU(75,imu,270.0,display,true);
        	if(getCurrImuYaw(imu)>270)
	  		turnLeft_waitOnIMU(30,imu,270.0,display,true);
        	else
			turnRight_waitOnIMU(30,imu,270.0,display,true);
		fwd_waitOnTOF(50,tof2,display,300);
		turnBackRight_waitOnIMU(40,imu,0.0,display,true);
		strafeLeft_timed(100,1000000,display);
		stop(50000,display);
		bwd_waitOnTOF(30,tof5,display,300);
		stop(50000,display);
		strafeLeft_timed(100,1000000,display);
		stop(50000,display);
		fwd_timed(20,350000,display);
		stop(50000,display);
		sendCommand(22,50,display);	// Spin wheel CCW
	*/
	}

	//for future purposes
	sendCommand(0,0,display);

	return(0);
}

