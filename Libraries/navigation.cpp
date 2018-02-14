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

void waitOnTOF(TOF *tof, int targetDistance)
{
    int currDistance;
    int targetMax=targetDistance+1;
    int targetMin=targetDistance-1;
    currDistance  = getDistance(tof);
    while (currDistance < targetMin || currDistance > targetMax)
    {
        currDistance  = getDistance(tof);
    }
    return;
}

void waitOnIMU(RTIMU *imu, char axis, double targetDegree)
{
    double current=0.0f;
    double stopTarget=fmod((current+targetDegree),360.0);
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
    stopTarget=fmod((current+targetDegree),360.0);
    double stopTargetMin=fmod((stopTarget+360.0),360.0) - 1.0;
    double stopTargetMax=fmod((stopTarget+360.0),360.0) + 1.0;
    while (current < stopTargetMin || current > stopTargetMax)
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
    }
    return;
}


//Stops the robot for the amount of time specified by int runtime (in milliseconds)
void stop(int runtime, int display){
    unsigned char direction = 0;									// Set direction to STOP
    //TMG added 1/31/18
    unsigned char speed = 0;
    RS232_SendByte(CPORT_NR, direction);							// Send direction
    //Speed on stop signal not declared
    //RS232_SendByte(CPORT_NR, speed);							// Send speed
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
    waitOnIMU(imu,axis,target);
}

// Moves the robot backward at a specified speed for a set amount of time
void bwd_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 2;												// Set direction to BWD											
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'BWD at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    usleep(runtime);
}

// MOves the robot backwards until TOF target distance reached
void bwd_waitOnTOF(unsigned char speed, TOF *sensor, int display, int target) {
    unsigned char direction = 2;
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'BWD at %d'\n", speed);
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
    printf("Sent to Arduino: 'BWD at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,axis,target);
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
    waitOnIMU(imu,axis,target);
}
    

// Strafes the robot left at a specified speed for a set amount of time
void strafeLeft_timed(unsigned char speed, int runtime, int display){
	unsigned char direction = 4;												// Set direction to STRAFE LEFT									
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
    waitOnIMU(imu,axis,target);
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

void turnRight_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display) {
    unsigned char direction = 5;
    //Set direction to TURN RIGHT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'TURN RIGHT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,'z',targetYaw);
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

void turnLeft_waitOnIMU(unsigned char speed, RTIMU *imu, double targetYaw, int display) {
    unsigned char direction = 6;
    //Set direction to TURN LEFT
    RS232_SendByte(CPORT_NR, direction);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: 'TURN LEFT at %d'\n", speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);
    waitOnIMU(imu,'z',targetYaw);
}

void sendCommand(unsigned char command, unsigned char speed, int display) {
    RS232_SendByte(CPORT_NR, command);
    RS232_SendByte(CPORT_NR, speed);
    printf("Sent to Arduino: %d, %d'\n", command, speed);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", command, speed);
}


void wallFollowDrive(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
void wallFollowStrafe(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, RTIMU *imu, int display);
