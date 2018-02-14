//Sensor package libarary WIP
/******************************
* File: SensorLib.h           *
* Author: Tommy Gonsewski     *
* Last Updated: 1/31/18	      *
*******************************
*/

#ifndef _SENSORLIB_H
#define _SENSORLIB_H


#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "inc/vl53l0x_api.h"
#include "inc/vl53l0x_platform.h"
#include "Libraries/RTIMULib2/RTIMULib/RTIMULib.h"
#include "Adafruit_VL6180X.h"

/* I2C Expander MUX */
struct mux {
    int muxStatus;
    char data_write[1];
    int initVal;
    int activeInput;
};
typedef struct mux MUX;

MUX *initMUX();
void switchMUX(MUX *,int);


/* Adafruit VL6180X */
struct shortRange {
    Adafruit_VL6180X vl;
    MUX *mux;
    int inputNo;
    uint8_t range;
    uint8_t status;
};

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
uint8_t getShortRange(SRANGE *);

/* Adafruit VL53L0X */
struct longRange {
    //VL53L0X vl53l0x;  Old implementation
    VL53L0X_Dev_t MyDevice;
    VL53L0X_Dev_t *pMyDevice;
    VL53L0X_Error Status; 
    VL53L0X_RangingMeasurementData_t *RangingMeasurementData;
    MUX *mux;
    int inputNo;
};

typedef struct longRange LRANGE;

//private declarations
void print_range_status(LRANGE *lrange);
VL53L0X_Error calibrateSingleMeasure(LRANGE *lrange);
VL53L0X_Error dataInitializeLR(LRANGE *lrange);


/* Method: initLongRange
    Description: Initializes VL53L0X at mux inputNo to long range mode
		w/ timing budget as specified in function call parameter
		"timingBudgetMicro"
*/
LRANGE *initLongRange(MUX *,int);
    
/* Method: getLongRange
    Description: Input multiplexor value as parameter and
		receive 16-bit unsigned val range value (mm)
*/
uint16_t getLongRange(LRANGE *);

/*TOF*/
struct tof {
    SRANGE *srange;
    LRANGE *lrange;
    int isLRANGE;
    MUX *mux;
    int inputNo;
};

typedef struct tof TOF;
TOF *newTOF(int ,MUX *,int);
int isLRANGE(TOF *);
int getDistance(TOF *tof);

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

struct Command
{
    int cmdVal;
    unsigned long cmdUSec;
};

typedef struct Command cmd;

cmd *addCmd(int initVal, unsigned long initUSec);
int binaryToDecimal(const char *);
char *decodeSignal(cmd **cmdArr,int maxToggles);
int routeread();

#endif
