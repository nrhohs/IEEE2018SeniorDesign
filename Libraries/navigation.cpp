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
#include "RTIMULib2\RTIMULib\RTIMULib.h"
#include "RTIMULib2\RTIMULib\RTMath.h"
#include "navigation.h"
#include "rs232.h"

#define BUF_SIZE 8

//Stops the robot for the amount of time specified by int runtime (in milliseconds)
void stop(int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

    str_send[0] = 0;												// Set direction to STOP
    RS232_SendByte(cport_nr,str_send[0]);							// Send direction
    RS232_SendByte(cport_nr,str_send[1]);							// Send speed
    printf("Sent to Arduino: 'STOP for %d'\n", runtime);			// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);	// LCD output
    usleep(runtime);												// Wait for X milliseconds
}

void stop_readIMU();
void stop_readTOF();

// Moves the robot forward at a specified speed for a set amount of time
void fwd_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 1;												// Set direction to FWD
	str_send[1]	= speed;											// Set speed
    RS232_SendByte(cport_nr,str_send[0]);							// Send direction
    RS232_SendByte(cport_nr,str_send[1]);							// Send speed
    printf("Sent to Arduino: 'FWD at %d for %d '\n", speed, runtime);// Cmdline output of command
    lcdPosition(display,0,0);										// Reset position of lcd cursor
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);	// LCD output
    usleep(runtime);												// Move for X time
}

void fwd_waitOnTOF();
void fwd_waidOnIMU();

// Moves the robot backward at a specified speed for a set amount of time
void bwd_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 2;												// Set direction to BWD
	str_send[1]	= speed;											
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: 'BWD at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(runtime);
}

void bwd_waitOnTOF();
void bwd_waitOnIMU();

// Strafes the robot right at a specified speed for a set amount of time
void strafeRight_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 3;												// Set direction to STRAFE RIGHT
	str_send[1]	= speed;											
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: 'STRAFE RIGHT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(runtime);
}

void strafeRight_waitOnTOF();
void strafeRight_waitOnIMU();

// Strafes the robot left at a specified speed for a set amount of time
void strafeLeft_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 4;												// Set direction to STRAFE LEFT
	str_send[1]	= speed;											
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: 'STRAFE LEFT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(runtime);
}

void strafeLeft_waitOnTOF();
void strafeLeft_waitOnIMU();

// Turns the robot right about it's center axis at a specified speed for a set amount of time
void turnRight_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 5;												// Set direction to TURN RIGHT
	str_send[1]	= speed;											
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: 'TURN RIGHT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(runtime);
}

void turnRight_waitOnTOF();
void turnRight_waitOnIMU();

// Turns the robot left about it's center axis at a specified speed for a set amount of time
void turnLeft_timed(unsigned char speed, int runtime){
	unsigned char str_send[BUF_SIZE];
    int cport_nr = 24;
    int display = lcdInit(2,16,4,100,102,103,104,105,106,0,0,0,0);

	str_send[0] = 6;												// Set direction to TURN LEFT
	str_send[1]	= speed;											
    RS232_SendByte(cport_nr,str_send[0]);
    RS232_SendByte(cport_nr,str_send[1]);
    printf("Sent to Arduino: 'TURN LEFT at %d for %d '\n", speed, runtime);
    lcdPosition(display,0,0);
    lcdPrintf(display,"Sent: '%3d %3d'", str_send[0],str_send[1]);
    usleep(runtime);
}

void turnLeft_waitOnTOF();
void turnLeft_waitOnIMU();

void wallFollow();
