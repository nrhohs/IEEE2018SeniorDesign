//Sensor package libarary WIP

#ifndef _SENSORLIB_H
#define _SENSORLIB_H


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

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
unit8_t *initVL6180X(MUX *,int);
/* Method: getShortRange
    Description: Input multiplexor value as parameter and
    		receive 8-bit unsigned val range value (mm)
*/
uint8_t getShortRange(SRANGE *,MUX *,int);

//Adafruit VL53L0X
struct longRange {
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice = &MyDevice;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
};
#typdef struct longRange LRANGE;
VL53L0X_Error  initLongRange(LRANGE *,MUX *,int);
    
/* Method: getLongRange
    Description: Input multiplexor value as parameter and
		receive 16-bit unsigned val range value (mm)
*/
uint16_t getLongRange(LRANGE *, MUX *,int);


#endif
