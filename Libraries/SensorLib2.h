///Sensor package libarary WIP
/******************************
* File: SensorLib.h           *
* Author: Tommy Gonsewski     *
* Last Updated: 1/17/18	      *
*******************************
*/

#ifndef _SENSORLIB_H
#define _SENSORLIB_H


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

/* I2C Expander MUX */
struct mux {
    int muxStatus;
    int initVal;
    int activeInput;
};
typedef struct mux MUX;

MUX *initMUX();
void switchMUX(MUX *,int);


/* Adafruit VL6180X */
typedef struct shortRange SRANGE;

SRANGE *initVL6180XwoMUX();

/* Method: initVL6180X
    Description: Initializes and returns short range TOF by providing
		the MUX and MUX input to function.
*/
SRANGE *initVL6180X(MUX *,int);
/* Method: getShortRange
    Description: Input multiplexor value as parameter and
    		receive 8-bit unsigned val range value (mm)
*/
uint8_t getShortRangewoMUX(SRANGE *);
uint8_t getShortRange(SRANGE *);

/* Adafruit VL53L0X */
typedef struct longRange LRANGE;

/* Method: initLongRange
    Description: Initializes VL53L0X at mux inputNo to long range mode
		w/ timing budget as specified in function call parameter
		"timingBudgetMicro"
*/
LRANGE *initLongRangewoMUX();
LRANGE *initLongRange(MUX *,int);
    
/* Method: getLongRange
    Description: Input multiplexor value as parameter and
		receive 16-bit unsigned val range value (mm)
*/
uint16_t getLongRangewoMUX(LRANGE *);
uint16_t getLongRange(LRANGE *);

/* IR Reader */
/*
 * Sets up/starts/reads from IR receiver
 *
 * This function returns the int value of the route for
 * later display on the LCD screen
 */
int readIRCode();

/* IMU */
/*
 * Initializer for IMU on software I2C
 *
 * Function returns an RTIMU object
 */
RTIMU *imuInit();
/*
 * Retrieve roll from IMU as double
 *
 */
double getCurrImuRoll(RTIMU *imu);
/*
 * Retrieve pitch from IMU as double
 *
 */
double getCurrImuPitch(RTIMU *imu); 
/*
 * Retrieve yaw from IMU as double
 *
 */
double getCurrImuYaw(RTIMU *imu);
/*
 * Retriever pitch from IMU as double
 * 
 */
void waitOnImuYaw(RTIMU *imu, double start, Adafruit_VL6180X *vl);


#endif
