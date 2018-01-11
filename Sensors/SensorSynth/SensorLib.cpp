#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include "Adafruit_VL6180X.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "SensorLib.h"

//Private Helper prototypes
//I2C Expander
int getMUXStatus(MUX *);
void setMUXStatus(MUX *, int);
int getMUXInput(MUX *);
void setMUXInput(MUX *, int);

//I2C Expander MUX
MUX *initMUX() {
    MUX *newMUX = malloc(sizeof(MUX));
    char buf[15];
    int muxStatus = open(buf,0_RDWR);
    if (ioctl(muxStatus,I2C_SLAVE,0x70)<0) {
	printf("MUX initialization failed\n");
	setMUXStatus(newMUX,-1);
	return newMUX;
    }
    else {
	setMUXStatus(newMUX,0);
	switchMUX(newMUX,0);
	return newMUX;
    }
}

void switchMUX(MUX *mux, int inputNo) {
    if(!getMUXStatus(mux)) {
        char data_write[1];
        data_write[0] = 1<<inputNo;
        write(mux, data_write, 1);
	setMUXInput(mux,1);
    }
    else {
	printf("Cannot switch MUX, status error\n");
    }
}

//VL6180X
//Assumes the TOF is connected by MUX
SRANGE *initVL6180X(SRANGE *,MUX *mux,int inputNo) {
    SRANGE *newSRANGE = malloc(sizeof(SRANGE));
    newSRANGE->vl = Adafruit_VL6180X();
    return newSRANGE;
}
SRANGE *getShortRange(SRANGE &srange,MUX *mux,int inputNo) {
    if (!getMUXStatus(mux)) {
	//switch MUX input if needed
	if (getMUXInput(mux)!=inputNo)
	    switchMUX(mux,inputNo);
	srange->range=srange->vl.readRange();
	srange->status=srange->vl.readStatus();
    }
    else {
	srange->range=-1;
	srange->status=-1;
    }
    return srange->range;
}

VL53L0X_Error initLongrange(VL53L0X_Dev_t *pMyDevice, MUX *mux, int inputNo) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

   //Static initialization
    if (Status == VL53L0X_ERROR_NONE) {
	printf("Performing Static Init...\n");
    	Status = VL53L0X_StaticInit(pMyDevice);
    }

    //RefCalibartion
    if (Status == VL53L0X_ERROR_NONE) {
   	printf("Performing Refrence Calibration...\n");
 	Status = VL53L0X_PerformRefCalibration(pMyDevice,



uint16_t getLongRange(LRANGE *lrange, MUX *mux,int inputNo) {
    if (!getMUXStatus(mux)) {
	//switch MUX input if needed
	if (getMUXInput(mux)!=inputNo)
	    switchMUX(mux,inputNo);
	lrange->status=
		VL53L0X_PerformSingleRangingMeasurement(lrange->pMyDevice,&lrange->RangingMeasurementData);
    }
    return lrange->RangingMeasurmentData.RangeMilliMeter;
}
	

//Private Helper methods
int getMUXStatus(MUX *mux) {
    return mux->initVal;
}
void setMUXStatus(MUX *mux, int val) {
    mux->initVal=val;
}
int getMUXInput(MUX *mux) {
    return mux->activeInput;
}
void setMUXInput(MUX *mux, int val) {
    mux->activeInput=val;
}



