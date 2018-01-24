//Sensor package libarary WIP
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
#include "Adafruit_VL6180X.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

//I2C Expander MUX
struct mux {
    int initVal;
    int activeInput;
};
#typedef struct mux MUX;

void initMUX();
void switchMUX(MUX *,int);


//Adafruit VL6180X
struct shortRange {
    Adafruit_VL6180X vl;
    uint8_t range;
    uint8_t status;
};
#typedef struct shortRange SRANGE;

/* Method: initVL6180X
    Description: Initializes and returns short range TOF by providing
		the MUX and MUX input to function.
*/
SRANGE *initVL6180X(MUX *,int);
/* Method: getShortRange
    Description: Input multiplexor value as parameter and
    		receive 8-bit unsigned val range value (mm)
*/
void getShortRange(SRANGE *,MUX *,int);

//Adafruit VL53L0X
struct longRange {
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice;
    VL53L0X_Error Status;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
};
#typdef struct longRange LRANGE;

/* Method: initLongRange
    Description: Initializes VL53L0X at mux inputNo to long range mode
		w/ timing budget as specified in function call parameter
		"timingBudgetMicro"
*/
LRANGE *initLongRange(MUX *,int,int);
    
/* Method: getLongRange
    Description: Input multiplexor value as parameter and
		receive 16-bit unsigned val range value (mm)
*/
LRANGE *getLongRange(LRANGE *, MUX *,int);

ledRead();


#endif
