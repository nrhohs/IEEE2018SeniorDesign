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
#include "ledRead.h"
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
SRANGE *initVL6180X(MUX *mux,int inputNo) {
    SRANGE *newSRANGE = malloc(sizeof(SRANGE));
    newSRANGE->vl = Adafruit_VL6180X();
    return newSRANGE;
}
SRANGE *getShortRange(SRANGE *srange,MUX *mux,int inputNo) {
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

LRANGE *initLongrange(MUX *mux, int inputNo, 
	int timeBudgetMicro) {
    LRANGE *newLRANGE=malloc(sizeof(LRANGE));
    newLRANGE->Status=VL53L0X_ERROR_NONE; 
    newLRANGE->pMyDevice = &newLRANGE->MyDevice;
    VL53L0X_Error status = newLRANGE->Status;

   //Static initialization
    if (status == VL53L0X_ERROR_NONE) {
	printf("Performing Static Init...\n");
    	status = VL53L0X_StaticInit(pMyDevice);
    }

    //RefCalibration
    if (status == VL53L0X_ERROR_NONE) {
   	printf("Performing Refrence Calibration...\n");
	//Initialize device
 	status = VL53L0X_PerformRefCalibration(pMyDevice,
		&VhvSettings, &PhaseCal);
    }

    if (status == VL53L0X_ERROR_NONE) {
	printf("Performing Spad Management...\n");
	//Device InitializatioN
	status=VL53L0X_PerformRefSpadManagement(pMyDevice,
		&refSpadCount, &isApertureSpads);
    }

    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetLimitCheckEnable(pMyDevice,
		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetLimitCheckEnable(pMyDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetLmitCheckValue(pMyDevice,
		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
		(FixPoint1616_t)(60*65536));
    }

    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice,
		timeBudgetMicro);
    }
    
    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
		VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
    }
	
    if (status == VL53L0X_ERROR_NONE) {
	status = VL53L0X_SetVcselPulsePeriod(pMyDevice,
		VL53LOX_VCSEL_PERIOD_FINAL_RANGE, 14);
    }
    printf("Long rang initialization complete...\n");
    newLRANGE->status = Status;
    return newLRANGE; 
}


LRANGE *getLongRange(LRANGE *lrange, MUX *mux,int inputNo) {
    if (!getMUXStatus(mux)) {
	//switch MUX input if needed
	if (getMUXInput(mux)!=inputNo)
	    switchMUX(mux,inputNo);
	lrange->status=
		VL53L0X_PerformSingleRangingMeasurement(lrange->pMyDevice,&lrange->RangingMeasurementData);
    }
    return lrange;
}


//IRLED READER
ledRead();

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



