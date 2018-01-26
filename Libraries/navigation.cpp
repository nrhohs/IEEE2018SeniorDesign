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
#include "Adafruit_VL6180x.h"
#include "RTIMULib.h"
#include "RTMath.h"
#include "navigation.h"
#include "rs232.h"
#include "SensorLib2.h"

#define CPORT_NR 24

//Stops the robot for the amount of time specified by int runtime (in milliseconds)
void stop(int runtime, int display){
    unsigned char direction = 0;									// Set direction to STOP
    RS232_SendByte(CPORT_NR, direction);							// Send direction
    RS232_SendByte(CPORT_NR, speed);							// Send speed
    printf("Sent to Arduino: 'STOP for %d'\n", runtime);			// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", direction, speed);	// LCD output
    usleep(runtime);												// Wait for X milliseconds
}

void stop_readTOF(TOF *sensor, int display);

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

void fwd_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void fwd_waidOnIMU(unsigned char speed, IMU *imu, int display);

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

void bwd_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void bwd_waitOnIMU(unsigned char speed, IMU *imu, int display);

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

void strafeRight_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void strafeRight_waitOnIMU(unsigned char speed, IMU *imu, int display);

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

void strafeLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void strafeLeft_waitOnIMU(unsigned char speed, IMU *imu, int display);

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

void turnRight_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void turnRight_waitOnIMU(unsigned char speed, IMU *imu, int display);

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

void turnLeft_waitOnTOF(unsigned char speed, TOF *sensor, int display);
void turnLeft_waitOnIMU(unsigned char speed, IMU *imu, int display);

void wallFollowDrive(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, IMU *imu, int display);
void wallFollowStrafe(unsigned char speed, unsigned char direction, TOF *face, TOF *sideOne, TOF *sideTwo, IMU *imu, int display);