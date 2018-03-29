/**********************************************************
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

	//printf("%d, %s, %s", argc, argv[0], argv[1]);
	
	//Initialize gpio
	if(gpioInitialise()<0) return 1;

	//Initialize Stop Switch
	/*Stop Switch Not Implemented Yet (future)
	gpioSetMode(21, PI_INPUT);
	gpioSetPullUpDown(21, PI_PUD_UP);
	gpioSetISRFunc(21, FALLING_EDGE, -1, STOPISR);
	*/

	//Initializes MUX
	MUX *mux = initMUX();

	//Initializes TOFs
	TOF *tof0 = newTOF(0, mux, 0);	//front facing SR
	TOF *tof1 = newTOF(0, mux, 1);	//right facing SR (front)
	TOF *tof2 = newTOF(1, mux, 2);	//right facing SR (back)
	TOF *tof3 = newTOF(1, mux, 3);	//back facing SR
	TOF *tof4 = newTOF(0, mux, 4);	//left facing SR  (back)
	TOF *tof5 = newTOF(1, mux, 5);	//left facing SR  (front)
	TOF *tof6 = newTOF(1, mux, 6);	//right facing LR (box TOF)
	//    TOF *tof7 = newTOF(1, mux, 7);	//front facing LR (dropoff TOF)

	//Initializes IMU
	RTIMU *imu = imuInit();

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

	int rawcode = 256;
	//Initialize rawcode > 8 to enter  positioning loop (was 0)

	//Display initial Sensor Outputs
	softPwmWrite(0,100);
	softPwmWrite(2,100);
	softPwmWrite(3,0);

	printf("TOF0: %d  ",getDistance(tof0));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF0: %d  ",getDistance(tof0));
	printf("TOF1: %d  ",getDistance(tof1));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF1: %d  ",getDistance(tof0));
	printf("TOF2: %d  ",getDistance(tof2));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF2: %d  ",getDistance(tof0));
	printf("TOF3: %d  ",getDistance(tof3));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF3: %d  ",getDistance(tof0));
	printf("TOF4: %d  ",getDistance(tof4));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF4: %d  ",getDistance(tof0));
	printf("TOF5: %d  ",getDistance(tof5));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF5: %d  ",getDistance(tof0));
	printf("TOF6: %d  ",getDistance(tof6));
	lcdPosition(display,0,0);
	lcdPrintf(display,"TOF6: %d  ",getDistance(tof0));
	while(getCurrImuYaw(imu)==0);
	printf("IMU:  %f\n",getCurrImuYaw(imu));
	lcdPosition(display,0,0);
	lcdPrintf(display,"IMU:  %5f\n",getCurrImuYaw(imu));

	softPwmWrite(0,0);
	softPwmWrite(2,100);
	softPwmWrite(3,0);


	//provides sensor readouts
	if (argc>1){
		for (int i=0;i<argc;i++) {
			//Repeatedly display sensor readings, no motor functionality
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
					printf("IMU:  %f,%f\n",getCurrImuYaw(imu),getCurrImuPitch(imu));
				}
			}
			//Hardcode the route from user input
			else if (strcmp(argv[i],"-route")==0){
				rawcode=argv[i+1][0]-'0';
			}

			//Currently (3/20/2018) being used as a debugging flag
			//for the latter stages of Robot testing
			else if (strcmp(argv[i],"-pid")==0){
	sendCommand(24,40,display);
	usleep(2000000);
	sendCommand(23,40,display);
	stop(500000,display);
	//			drive_PID(50,tof5,display,100);
				return 0;
				/*
				strafeLeft_wallFollow(30,imu,tof4,tof2,0.0,50,display);
				turn_IMUcorrection(imu,0.0,display);
				fwd_wallFollow(30,imu,tof1,tof4,0.0,100,display);
				sendCommand(4,30,display);
				while(getDistance(tof4)>30);
				//strafeLeft_waitOnTOF(30,tof4,display,40);
				stop(100000,display);
				sendCommand(22,50,display);
				
				return 0;
				sendCommand(24,40,display);
				usleep(2000000);
				sendCommand(23,40,display);
				usleep(2000000);
				stop(500000,display);
				//Actuator Test
				turnRight_waitOnIMU(75,imu,90.0,display,true);
				fwd_waitTOFwIMU(30,imu,90.0,tof5,300,display);
				fwd_waitOnIMU(200,imu,'y',10.0,display);
				fwd_waitOnTOF(100,tof5,display,500);
				stop(500000,display);

				driveToTOF(20,tof5,display,150);
				//bwd_waitOnTOF(50,tof5,display,150);
				stop(1000000,display);
				turnLeft_waitOnIMU(75,imu,0.0,display,true);
				stop(500000,display);
				turn_IMUcorrection(imu,0.0,display);
				fwd_waitOnTOF(50,tof5,display,85);
				return 0;
				*/
			}
		}
	}

	//Retrieve route information from IRBlaster 
	else {
		softPwmWrite(0,100);
		softPwmWrite(2,50);
		softPwmWrite(3,0);
		while (rawcode>8) {
	  	  rawcode = routeread(display);
		  lcdPosition(display,0,0);
		  lcdPrintf(display,"Positioning: %d  ",rawcode);
		}
		lcdClear(display);
		lcdPosition(display,0,1);
		lcdPrintf(display,"Route: %d",rawcode+1);
		softPwmWrite(0,0);
		softPwmWrite(2,100);
		softPwmWrite(3,0);
	}

	//Display initial Sensor Outputs


	/*----------------------First Task, Go to Button on Top of Ramp--------------------*/	
	if (rawcode >= 4){
		printf("run A");
		fwd_waitOnTOF(55, tof5, display, 85);
		fwd_timed(60, 200000,  display);
		stop(100000,display);
		bwd_waitOnTOF(55, tof5, display, 415);
		stop(100000,display);
		driveToTOF(20,tof5,display,470,1);

	}
	else{
		printf("run B");
		bwd_waitOnTOF(55,tof2,display,85);
		bwd_timed(60, 200000,  display);
		stop(100000,display);
		fwd_waitOnTOF(55, tof2, display, 415);
		stop(100000,display);
		driveToTOF(20,tof5,display,480,1);
	}
	/*----------------------First Task, Go to Button on Top of Ramp--------------------*/	

	/*------------Second Task, Go Down Ramp, Then Press button on main arena----------*/	
	//Rotate 90 to go down ramp
	turnLeft_waitOnIMU(75,imu,85.0,display,false);
	stop(200000,display);
	turn_IMUcorrection(imu,270.0,display);

	//Go down ramp
	fwd_timed(30,5000000,display);
	stop(1000000,display);

	//Reorient the Robot 
	turn_IMUcorrection(imu,270.0,display);

	//Go forward until box is found
	fwd_waitOnTOF(75,tof5,display,400);
	stop(200000,display);
	//go backward for brief time
	bwd_timed(40,1000000,display);


	//Rotate 90 to orient flag mech
	turnRight_waitOnIMU(75,imu,0.0,display,true);
	stop(200000,display);
	turn_IMUcorrection(imu,0.0,display);

	if (rawcode%4 < 2){
		//B button 0
		/************ Destination B *************************/
		bwd_waitOnTOF(50,tof2,display,160);
		bwd_timed(60,200000,display);
		stop(1000000,display);

		fwd_timed(40,900000,display);
		stop(100000,display);

		turn_IMUcorrection(imu,0.0,display);

		/***************** Flag Wheel **************************/

		printf("\n-------------Going into Flag Wheel----------\n");

		stop(100000,display);
		strafeLeft_timed(30,2000000,display);
		stop(100000,display);

		strafeLeft_wallFollow(30,imu,tof4,tof2,0.0,30,display);
		stop(100000,display);
		turn_IMUcorrection(imu,0.0,display);
		fwd_wallFollow(30,imu,tof1,tof4,0.0,100,display);
		sendCommand(4,30,display);
		while(getDistance(tof4)>25);
		//turn_IMUcorrection(imu,0.0,display);
		turnToFlag(imu,358.0,display);
		stop(100000,display);

		sendCommand(22,50,display);
	}
	else {
		//B button 1
		/****************** Destination B ***********************/
		fwd_waitOnTOF(50,tof5,display,160);
		fwd_timed(60,200000,display);
		stop(100000,display);

		bwd_timed(40,900000,display);

		stop(1000000,display);
		turn_IMUcorrection(imu,0.0,display);

		/***************** Flag Wheel **************************/
		printf("\n-------------Going into Flag Wheel----------\n");

		stop(100000,display);
		strafeLeft_timed(30,2000000,display);
		stop(100000,display);

		strafeLeft_wallFollow(30,imu,tof4,tof5,0.0,30,display);
		stop(100000,display);
		turn_IMUcorrection(imu,0.0,display);
		bwd_wallFollow(30,imu,tof0,tof4,0.0,100,display);
		sendCommand(4,30,display);
		while(getDistance(tof4)>25);
		//turn_IMUcorrection(imu,0.0,display);
		turnToFlag(imu,1.0,display);
		stop(100000,display);

		sendCommand(22,50,display);

	}
	/*--------------------------Task 4 Pick up Treasure Chest------------------------*/



	strafeRight_waitOnTOF(60,tof3,display,180);
	stop(10000,display);

	turn_IMUcorrection(imu,0.0,display);
	stop(10000,display);

	driveToTOF(30,tof5,display,300,1);
	sendCommand(2,10,display);
	while(getDistance(tof6)>250);
	//bwd_waitTOFwIMU(10,imu,0.0,tof6,100,display);
	stop(10000,display);

	//Was 280 target
	strafeRight_waitOnTOF(30,tof3,display,350);
	strafeRight_timed(30,1000000,display);
	//strafeLeft_timed(15,50000,display);
	stop(10000,display);
	bwd_timed(20,125000,display);
	stop(10000,display);

	sendCommand(24,40,display);
	usleep(2000000);
	sendCommand(23,40,display);
	stop(500000,display);
	usleep(4000000);

	// Check positioning
	//FRONT
	int front = getDistance(tof5);
	//BACK
	int back = getDistance(tof2);
	if (!(front < 8191 && back < 8191))
	    //TOF correction
	    driveToTOF(30,tof5,display,400,1);

	turnRight_waitOnIMU(75,imu,90.0,display,true);


	/*------------------------Final Task, Climb Plank and Press Button--------*/	

	fwd_waitTOFwIMU(30,imu,90.0,tof5,300,display);
	fwd_waitOnIMU(200,imu,'y',10.0,display);
	fwd_waitOnTOF(100,tof5,display,500);
	stop(500000,display);

	driveToTOF(20,tof5,display,150,1);
	//bwd_waitOnTOF(50,tof5,display,150);
	stop(500000,display);
	turnLeft_waitOnIMU(75,imu,0.0,display,true);
	stop(200000,display);
	turn_IMUcorrection(imu,0.0,display);
	if (rawcode%2==1) {
	    fwd_waitOnTOF(50,tof5,display,85);
	    fwd_timed(60,200000,display);
	}
	else {
	    bwd_waitOnTOF(50,tof2,display,85);
	    bwd_timed(60,200000,display);
	}


	//for future purposes
	sendCommand(0,0,display);

	return(0);
}

