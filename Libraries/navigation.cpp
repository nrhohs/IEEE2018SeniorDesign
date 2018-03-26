/********************************************************
					Navigation Library
This library is meant to be a set of intuitive navigation
functions that can be called by the main Robot.cpp file.
The purpose of this library is to make it easier to tweak
how the robot traverses the arena.
********************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <lcd.h>
#include <mcp23017.h>
#include <softPwm.h>
#include <unistd.h>
#include <wiringPi.h>
#include "RTIMULib.h"
#include "RTMath.h"
#include "navigation.h"
#include "rs232.h"
#include "SensorLib2.h"
#include <math.h>

#define CPORT_NR 24
#define PRINT_DEBUG 0

int pollTOF(TOF *tof, int targetDistance)
{
    int threshhold=0;
    int doneFlag=0;
    int currDistance;
    int targetMax=targetDistance+threshhold;
    int targetMin=targetDistance-threshhold;
    currDistance=getDistance(tof);
    printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
    printf("TOF%d target dist: %d\n",tof->inputNo,targetDistance);
    currDistance = getDistance(tof);
    if (targetDistance-currDistance >= 0)
    {
	if (currDistance > targetMin || currDistance > 8000) {
	    doneFlag=0;
	}
	else {
	    doneFlag=1;
	}
    }
    else
    {
	if (currDistance < targetMax && currDistance <= 8000)	{
	    doneFlag=1;
	}
	else
	{
	    doneFlag=0;
	}
    }
    if (PRINT_DEBUG)
    	printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
    return doneFlag;
}

void waitOnTOF(TOF *tof, int targetDistance)
{
    int threshhold=0;
    int currDistance;
    int targetMax=targetDistance+threshhold;
    int targetMin=targetDistance-threshhold;
    currDistance  = getDistance(tof);
    printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
    printf("TOF%d target dist: %d\n",tof->inputNo,targetDistance);
    if (targetDistance-currDistance>=0)
	//max range limit
    {
	while (currDistance < targetMin || currDistance>8000)
	{
	    currDistance = getDistance(tof);
	    if (PRINT_DEBUG)
	    	printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
	}
    }
    else
	// min range limit
    {
	while (currDistance > targetMax || currDistance>8000)
	{
	    currDistance = getDistance(tof);
	    if (PRINT_DEBUG)
	    	printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
	}
    }
/*
    while (currDistance < targetMin || currDistance > targetMax)
    {
        currDistance  = getDistance(tof);
	//printf("distance: %d\n",currDistance);
    }
*/
    printf("TOF%d current dist: %d\n",tof->inputNo,currDistance);
    return;
}

void waitOnIMU(RTIMU *imu, char axis, double targetDegree,bool isAbsolute)
{
    double threshhold = 3.0;
    double current=0.0f;
    double stopTarget=0.0f;
    switch (axis)
    {
	case 'x':
	    current=getCurrImuRoll(imu);
	    break;
	case 'y':
	    current=getCurrImuPitch(imu);
	    break;
	case 'z':
	    current=getCurrImuYaw(imu);
	    break;
	default:
	    printf("Invalid axis");
	    return;
    }
    printf("\n Current Position: %c %f\n",axis,current);
    if (!isAbsolute)
    	stopTarget=fmod((current+targetDegree+360),360.0);
    else
	stopTarget=targetDegree;
    printf("\n Stop Target: %c %f\n",axis,stopTarget);
    //double stopTargetMin=fmod((stopTarget),360.0) -  threshhold;
    //double stopTargetMax=fmod((stopTarget),360.0) + threshhold;
    double stopTargetMin=fmod((fmod(stopTarget,360.0) -  threshhold + 360),360.0);
    double stopTargetMax=fmod((fmod(stopTarget,360.0) +  threshhold + 360),360.0);
    if (stopTargetMin > 350.0 && axis=='z')
	stopTargetMax+=360.0;
    if (current < 30.0 && axis=='z')
	current+=360.0;
    // Threshhold implementation
    while (!(current < stopTargetMax && current > stopTargetMin)||current==-1)
    {
	if(PRINT_DEBUG)
	    printf("current Yaw %f\n",current);
    	switch (axis)
    	{
	    case 'x':
	    	current=getCurrImuRoll(imu);
	    	break;
	    case 'y':
	    	current=getCurrImuPitch(imu);
	    	break;
	    case 'z':
	    	current=getCurrImuYaw(imu);
	        break;
        }
    }
    // max only implementation
    /*
    if (stopTarget-current > 0)
    {
	while (current <= stopTarget || current==-1)
	{
    	switch (axis)
    	{
	    case 'x':
	    	current=getCurrImuRoll(imu);
	    	break;
	    case 'y':
	    	current=getCurrImuPitch(imu);
	    	break;
	    case 'z':
	    	current=getCurrImuYaw(imu);
	        break;
        }
		printf("current Yaw %f\n",current);
	}
    }
    else
    {
    while (current >= stopTargetMax || current==-1)
    {
    	switch (axis)
    	{
	    case 'x':
	    	current=getCurrImuRoll(imu);
	    	break;
	    case 'y':
	    	current=getCurrImuPitch(imu);
	    	break;
	    case 'z':
	    	current=getCurrImuYaw(imu);
	        break;
        }
	printf("current Yaw %f\n",current);
    }
    }
    */
    printf("current Yaw %f\n",current);
    return;
}


//Stops the robot for the amount of time specified by int runtime (in milliseconds)
void stop(int runtime, int display){
    unsigned char direction = 0;									// Set direction to STOP
    //TMG added 1/31/18
    unsigned char speed = 0;
    RS232_SendByte(CPORT_NR, direction);							// Send direction
    RS232_SendByte(CPORT_NR, speed);							// Send speed
    printf("Sent to Arduino: 'STOP for %d'\n", runtime);			// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    usleep(runtime);												// Wait for X milliseconds
}

int stop_readTOF(TOF *sensor, int display) {
    unsigned char direction = 0;									// Set direction to STOP
    //TMG added 1/31/18
    unsigned char speed = 0;
    RS232_SendByte(CPORT_NR, direction);							// Send direction
    RS232_SendByte(CPORT_NR, speed);							// Send speed
    printf("Sent to Arduino: 'STOP and poll '\n");			// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    //Simply poll sensor
    return getDistance(sensor);
}

// Moves the robot forward at a specified speed for a set amount of time
void fwd_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 1;									// Set direction to FWD
    RS232_SendByte(CPORT_NR, direction);								// Send direction
    RS232_SendByte(CPORT_NR, speed);							// Send speed
    printf("Sent to Arduino: 'FWD at %d for %d '\n", speed, runtime);// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    usleep(runtime);												// Move for X time
}

void fwd_waitOnTOF(unsigned char speed, TOF *sensor, int display, int target) {
    unsigned char direction = 1;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'FWD at %d '\n", speed);// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    //if target==NULL default target for TOF is 100
    if (target==100)
        waitOnTOF(sensor);
    else
	waitOnTOF(sensor,target);
}
    
void fwd_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display) {
    unsigned char direction = 1;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'FWD at %d'\n", speed);// Cmdline output of command
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    waitOnIMU(imu,axis,target,false);
}

//fwd_waitOnTOF w/ IMU correction
void fwd_waitTOFwIMU(unsigned char speed,RTIMU *imu,double IMUtarget,TOF *sensor,int TOFtarget,int display)
{
    double threshhold = 1.0;
    double current=0.0f;
    double stopTarget=0.0f;
    current=getCurrImuYaw(imu);
    //printf("\n Current Position: %f\n",current);
    stopTarget=IMUtarget;
    //printf("\n Stop Target: %f\n",stopTarget);
    double stopTargetMin=fmod((fmod(stopTarget,360.0) -  threshhold + 360),360.0);
    //printf("\n Stop target (min) = %f",stopTargetMin);
    double stopTargetMax=fmod((fmod(stopTarget,360.0) +  threshhold + 360),360.0);


    if (stopTargetMin > 350.0)
	stopTargetMax+=360.0;
    if (current < 30.0)
	current+=360.0;
    //printf("\n Stop target (max) = %f",stopTargetMax);

    //Send cmd to arduino
    unsigned char direction = 1;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'FWD at %d'\n", speed);// Cmdline output of command
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    //Check TOF
    while (!pollTOF(sensor,TOFtarget))
    {
    	current=getCurrImuYaw(imu);
    	if (current < 30.0)
		current+=360.0;
	//Correct left
    	//printf("\n current Yaw = %f",current);
	if (current > stopTargetMax)
	{
            turnLeft_waitOnIMU(25, imu, IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	    
	}
	//Correct right
	else if (current < stopTargetMin)
	{
	    turnRight_waitOnIMU(25,imu,IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	}
	else
	    continue;
    }
}

// Moves the robot backward at a specified speed for a set amount of time
void bwd_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 2;												// Set direction to BWD											
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'BWD at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

// MOves the robot backwards until TOF target distance reached
void bwd_waitOnTOF(unsigned char speed, TOF *sensor, int display, int target) {
    unsigned char direction = 2;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'BWD at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    if (target==100)
    	waitOnTOF(sensor);
    else
	waitOnTOF(sensor,target);
}

// Moves the robot backwards until IMU target degre on target axis acquired
//TMG function needs target degree and axis for IMU
void bwd_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display) {
    unsigned char direction = 2;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'BWD at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,axis,target,false);
}

//bwd_waitOnTOF w/ IMU correction
void bwd_waitTOFwIMU(unsigned char speed,RTIMU *imu,double IMUtarget,TOF *sensor,int TOFtarget,int display)
{
    double threshhold = 1.0;
    double current=0.0f;
    double stopTarget=0.0f;
    current=getCurrImuYaw(imu);
    //printf("\n Current Position: %f\n",current);
    stopTarget=IMUtarget;
    //printf("\n Stop Target: %f\n",stopTarget);
    double stopTargetMin=fmod((fmod(stopTarget,360.0) -  threshhold + 360),360.0);
   // printf("\n Stop target (min) = %f",stopTargetMin);
    double stopTargetMax=fmod((fmod(stopTarget,360.0) +  threshhold + 360),360.0);
    if (stopTargetMin > 350.0)
	stopTargetMax+=360.0;
    if (current < 30.0)
	current+=360.0;
   // printf("\n Stop target (max) = %f",stopTargetMax);

    //Send cmd to arduino
    unsigned char direction = 2;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
   // printf("Sent to Arduino: 'FWD at %d'\n", speed);// Cmdline output of command
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    //Check TOF
    while (!pollTOF(sensor,TOFtarget))
    {
    	current=getCurrImuYaw(imu);
    	if (current < 30.0)
		current+=360.0;
    //	printf("\n current Yaw = %f",current);
	//Correct left
	if (current > stopTargetMax)
	{
            turnLeft_waitOnIMU(25, imu, IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	    
	}
	//Correct right
	else if (current < stopTargetMin)
	{
	    turnRight_waitOnIMU(25,imu,IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	}
	else
	    continue;
    }
}


// Strafes the robot right at a specified speed for a set amount of time
void strafeRight_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 3;												// Set direction to STRAFE RIGHT							
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE RIGHT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

void strafeRight_waitOnTOF(unsigned char speed, TOF *sensor, int display, int target) {
    unsigned char direction = 3;
    //Set direction to STRAFE RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE RIGHT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    if (target==100)
	waitOnTOF(sensor);
    else
	waitOnTOF(sensor,target);
}

void strafeRight_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display) {
    unsigned char direction = 3;
    //Set direction to STRAFE RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE RIGHT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,axis,target,false);
}
    

// Strafes the robot left at a specified speed for a set amount of time
void strafeLeft_timed(unsigned char speed, int runtime, int display){
    unsigned char direction = 4;									// Set direction to STRAFE LEFT	
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE LEFT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

void strafeLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display, int target) {
    unsigned char direction = 4;
    //Set direction to STRAFE LEFT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE LEFT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    if (target==100)
	waitOnTOF(sensor);
    else
	waitOnTOF(sensor,target);
}

void strafeLeft_waitOnIMU(unsigned char speed, RTIMU *imu, char axis, double target, int display) {
    unsigned char direction = 4;
    //Set direction to STRAFE LEFT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE LEFT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,axis,target,false);
}

void strafeLeft_wallFollow(unsigned char speed, RTIMU *imu, TOF *sensor, TOF *tof2, double angle, int TOFtarget, int display) {
    unsigned char direction = 4;
    //Set direction to STRAFE LEFT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'STRAFE LEFT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);


    double threshhold = 1.0;
    int TOFthreshhold = 15;
    double current=0.0f;
    double stopTarget=0.0f;
    double IMUtarget = stopTarget;
    current=getCurrImuYaw(imu);
    //printf("\n Current Position: %f\n",current);
    //printf("\n Stop Target: %f\n",stopTarget);
    double stopTargetMin=fmod((fmod(stopTarget,360.0) -  threshhold + 360),360.0);
    //printf("\n Stop target (min) = %f",stopTargetMin);
    double stopTargetMax=fmod((fmod(stopTarget,360.0) +  threshhold + 360),360.0);


    if (stopTargetMin > 350.0)
	stopTargetMax+=360.0;
    if (current < 30.0)
	current+=360.0;
    //printf("\n Stop target (max) = %f",stopTargetMax);

    //Send cmd to arduino
    //Check TOF
    while (!pollTOF(sensor,TOFtarget))
    {
    	current=getCurrImuYaw(imu);
    	if (current < 30.0)
		current+=360.0;
	//Correct left
    	//printf("\n current Yaw = %f",current);
	if (current > stopTargetMax)
	{
            turnLeft_waitOnIMU(25, imu, IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	}
	//Correct right
	else if (current < stopTargetMin)
	{
	    turnRight_waitOnIMU(25,imu,IMUtarget,display,true);
	    //stop(400,display);
    	    RS232_SendByte(CPORT_NR, direction);
    	    RS232_SendByte(CPORT_NR, speed);
	}
	else if (getDistance(tof2) > 100+TOFthreshhold) {
	    sendCommand(2,10,display); 
	    while(getDistance(tof2)>100+TOFthreshhold);
	    sendCommand(direction,speed,display); 
	}
	else if (getDistance(tof2) < 100-TOFthreshhold) {
	    sendCommand(1,10,display); 
	    while(getDistance(tof2)<=100-TOFthreshhold);
	    sendCommand(direction,speed,display); 
	}
	else
	    continue;
    }
}

// Turns the robot right about it's center axis at a specified speed for a set amount of time
void turnRight_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 5;												// Set direction to TURN RIGHT									
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'TURN RIGHT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

void turnRight_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute) {
    unsigned char direction = 5;
    //Set direction to TURN RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'TURN RIGHT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,'z',targetYaw,isAbsolute);
}


// Turns the robot left about it's center axis at a specified speed for a set amount of time
void turnLeft_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 6;												// Set direction to TURN LEFT										
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'TURN LEFT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

void turnLeft_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute) {
    unsigned char direction = 6;
    //Set direction to TURN LEFT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    if (!isAbsolute)
    	waitOnIMU(imu,'z',(-1)*targetYaw,isAbsolute);
    else
	waitOnIMU(imu,'z',targetYaw,isAbsolute);
}

void sendCommand(unsigned char command, unsigned char speed, int display) {
    RS232_SendByte(CPORT_NR, command);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: %d, %d'\n", command, speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", command, speed);
}

void sendCommandNoDisplay(unsigned char command, unsigned char speed) {
    RS232_SendByte(CPORT_NR, command);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: %d, %d'\n", command, speed);
}

void stopExit() {
    RS232_SendByte(CPORT_NR, 0);
    RS232_SendByte(CPORT_NR, 0);
    printf("Sent to Arduino: 0, 0'\n");
    printf("Stopping bot and exiting\n");
}

void turnBackLeft_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute) {
    unsigned char direction = 13;
    //Set direction to TURN RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'TURN BackLeft Corner at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,'z',targetYaw,isAbsolute);
} 

void turnBackRight_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display, bool isAbsolute) {
    unsigned char direction = 11;
    //Set direction to TURN RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'TURN BackLeft Corner at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,'z',targetYaw,isAbsolute);
} 

// Moves the robot backwards until TOF target distance reached
void bwdArcRight_waitOnTOF(unsigned char speed,unsigned char offset, TOF *sensor, int display, int target) {
    unsigned char direction = 2;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    //printf("Sent to Arduino: 'BWD at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    //printf("TESTING");

    direction = 20;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, offset);
    //printf("Sent to Arduino: 'Arc at %d'\n", offset);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);

    if (target==100)
    	waitOnTOF(sensor);
    else
		waitOnTOF(sensor,target);
}

void turn_IMUcorrection(RTIMU *imu, double angle, int display) {
    if (angle==0.0) {
    	if (getCurrImuYaw(imu)<180)
	    turnLeft_waitOnIMU(20,imu,angle,display,true);
    	else
	    turnRight_waitOnIMU(20,imu,angle,display,true);
    }
    else {
    	if (getCurrImuYaw(imu)>angle)
	    turnLeft_waitOnIMU(20,imu,angle,display,true);
    	else
	    turnRight_waitOnIMU(20,imu,angle,display,true);
    }
}

void driveToTOF(unsigned char speed, TOF *tof,int display,int target,bool dir) {
    int currDistance=getDistance(tof);
    if (target-currDistance >= 0) {
	if (dir)
	  bwd_waitOnTOF(speed,tof,display,target);
    	else
	  fwd_waitOnTOF(speed,tof,display,target);
    }
    else {
	if (dir)
	  fwd_waitOnTOF(speed,tof,display,target);
    	else
	  bwd_waitOnTOF(speed,tof,display,target);
    }
}    
	
	
void strafeToTOF(unsigned char speed, TOF *tof,int display,int target) {
    int currDistance=getDistance(tof);
    if (target-currDistance >= 0)
	strafeRight_waitOnTOF(speed,tof,display,target);
    else
	strafeLeft_waitOnTOF(speed,tof,display,target);
}    

void wallFollowDrive(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
void wallFollowStrafe(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
